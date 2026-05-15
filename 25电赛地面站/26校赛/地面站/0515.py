# -*- coding: utf-8 -*-
"""
无人机野生动物监测系统主程序。

功能：
1. 通过串口屏录入 3 个禁飞区坐标，并在屏幕上标记。
2. 调用外部 coverage_planner.CoveragePlanner 生成覆盖路径。
3. 输出旧工程兼容的 flight_path.txt、simplified_path.txt、router.txt。
4. 在串口屏上绘制路径。
5. 接收无人机航点帧和动物识别帧，更新串口屏显示。
"""

import ast
import time

import RPi.GPIO as GPIO
import serial
from coverage_planner import CoveragePlanner


# ========== 空地协同协议常量 ==========
AIR_PORT = "/dev/my_blue"
AIR_BAUDRATE = 9600
AIR_FRAME_HEADER = 0xAA
AIR_FRAME_TAIL = 0xFF
AIR_FRAME_LENGTH = 5
FORBIDDEN_FRAME_LENGTH = 8
DRAW_LINE_OFFSETS = (-1, 0, 1)
DRAW_LINE_DELAY = 0.02


# ========== 动物 ID 映射 ==========
ANIMAL_ID_MAP = {
    0x00: "老虎",
    0x01: "狼",
    0x02: "猴子",
    0x03: "孔雀",
    0x04: "大象",
    0xFF: "无动物",
}

ANIMAL_NAME_TO_ID = {
    "老虎": 0x00,
    "狼": 0x01,
    "猴子": 0x02,
    "孔雀": 0x03,
    "大象": 0x04,
    "无动物": 0xFF,
}

PREDEFINED_ANIMALS = {
    ("A6", "B3"): ("老虎", 1),
    ("A7", "B5"): ("老虎", 1),
    ("A8", "B4"): ("孔雀", 1),
    ("A9", "B7"): ("猴子", 1),
    ("A6", "B7"): ("大象", 1),
    ("A4", "B7"): ("狼", 1),
    ("A4", "B5"): ("狼", 2),
    ("A3", "B6"): ("大象", 1),
    ("A4", "B2"): ("孔雀", 1),
}


# ========== 单蓝牙空地协同帧解析 ==========
class AirProtocolFrameParser:
    """
    下行帧格式，固定 5 字节：
    [0xAA][waypoint_idx][animal_cls][animal_cnt][0xFF]

    cnt = 0 时表示飞行中/无新结果，只更新航点进度。
    cnt >= 1 时表示检测完成，cls 字段有效；cls = 0xFF 表示本航点无动物。
    """

    FRAME_HEADER = AIR_FRAME_HEADER
    FRAME_TAIL = AIR_FRAME_TAIL
    FRAME_LENGTH = AIR_FRAME_LENGTH

    def __init__(self):
        self.buffer = bytearray()

    def feed_data(self, data):
        """把串口新收到的数据加入缓冲区。"""
        self.buffer.extend(data)

    def parse_frame(self):
        """从缓冲区解析一帧，返回 (waypoint_idx, animal_cls, animal_cnt) 或 None。"""
        while True:
            header_idx = self.buffer.find(bytes([self.FRAME_HEADER]))
            if header_idx == -1:
                self.buffer.clear()
                return None

            if len(self.buffer) < header_idx + self.FRAME_LENGTH:
                self.buffer = self.buffer[header_idx:]
                return None

            frame = self.buffer[header_idx:header_idx + self.FRAME_LENGTH]

            if frame[4] != self.FRAME_TAIL:
                self.buffer = self.buffer[header_idx + 1:]
                continue

            self.buffer = self.buffer[header_idx + self.FRAME_LENGTH:]
            return frame[1], frame[2], frame[3]


# ========== 串口屏发送工具 ==========
def send_to_screen(cmd_text):
    """发送文本命令到串口屏，并自动追加串口屏结束符。"""
    ser2.write(cmd_text.encode("GB2312"))
    ser2.write(bytes.fromhex("ff ff ff"))
    ser2.flush()


def send_number_to_screen(var_name, value):
    """发送数值到串口屏变量，例如 n0.val=100。"""
    send_to_screen("{}.val={}".format(var_name, int(value)))


# ========== 路径文件兼容输出 ==========
def calculate_t(current_xy, previous_xy):
    """按旧 router.txt 规则计算两点之间的飞行时间 t。"""
    current_real = CoveragePlanner.xy_to_real(current_xy)
    previous_real = CoveragePlanner.xy_to_real(previous_xy)
    dx = abs(current_real[0] - previous_real[0])
    dy = abs(current_real[1] - previous_real[1])
    delta = max(dx, dy)
    t_value = delta / 0.5 * 3
    return int(round(t_value)) if t_value != 0 else 3


def save_flight_path(full_path, filename="flight_path.txt"):
    """保存旧工程使用的航点文本格式：001: (A9, B1)。"""
    with open(filename, "w", encoding="utf-8") as file:
        for idx, xy in enumerate(full_path, 1):
            file.write("%03d: %s\n" % (idx, CoveragePlanner.xy_to_ab_str(xy)))


def get_turning_points(full_path):
    """提取转折点，用于串口屏画简化路径。"""
    if len(full_path) <= 2:
        return list(full_path)

    points = [full_path[0]]
    for idx in range(1, len(full_path) - 1):
        previous_point = full_path[idx - 1]
        current_point = full_path[idx]
        next_point = full_path[idx + 1]
        previous_direction = (
            current_point[0] - previous_point[0],
            current_point[1] - previous_point[1],
        )
        next_direction = (
            next_point[0] - current_point[0],
            next_point[1] - current_point[1],
        )
        if previous_direction != next_direction:
            points.append(current_point)

    points.append(full_path[-1])
    return points


def save_simplified_path(full_path, filename="simplified_path.txt"):
    """保存转折点路径，格式保持为旧工程可读取的 001: (A9, B1)。"""
    turning_points = get_turning_points(full_path)
    with open(filename, "w", encoding="utf-8") as file:
        for idx, xy in enumerate(turning_points, 1):
            file.write("%03d: %s\n" % (idx, CoveragePlanner.xy_to_ab_str(xy)))
    return turning_points


def save_router_file(full_path, filename="router.txt", altitude=1.2):
    """保存旧工程兼容的 x,y,z,t,a,d,s 格式 router 指令。"""
    with open(filename, "w", encoding="utf-8") as file:
        file.write("x,y,z,t,a,d,s\n")
        file.write("0,0,%.1f,10,0,0,0\n" % altitude)

        previous = full_path[0]
        for idx, xy in enumerate(full_path):
            real_x, real_y = CoveragePlanner.xy_to_real(xy)
            t_value = 3 if idx == 0 else calculate_t(xy, previous)
            file.write("%.1f,%.1f,%.1f,%d,0,0,0\n" % (real_x, real_y, altitude, t_value))
            previous = xy

        file.write("-1,0,%.1f,18,0,0,0\n" % altitude)
        file.write("0,0,0.2,10,0,0,0\n")


def generate_flight_files(forbidden_area):
    """调用外部 CoveragePlanner，并生成后续主流程需要的 3 个路径文件。"""
    planner = CoveragePlanner(forbidden_area)
    strategy_name, full_path, steps = planner.plan()
    save_flight_path(full_path, "flight_path.txt")
    save_simplified_path(full_path, "simplified_path.txt")
    save_router_file(full_path, "router.txt")
    print("路径规划完成：策略=%s，步数=%d，航点数=%d" % (strategy_name, steps, len(full_path)))
    return full_path


def parse_path_file(file_path):
    """从 001: (A9, B1) 格式的路径文件中读取坐标元组。"""
    points = []
    with open(file_path, "r", encoding="utf-8") as file:
        for line in file:
            start = line.find("(")
            end = line.find(")")
            if start == -1 or end == -1 or end <= start:
                continue

            coord_str = line[start + 1:end]
            parts = [part.strip() for part in coord_str.split(",")]
            if len(parts) == 2:
                points.append((parts[0], parts[1]))
    return points


def format_coord_display(coord):
    """把 ('A9', 'B1') 格式化成串口屏显示用的坐标文本。"""
    return "（%s，%s）" % (coord[0].strip(), coord[1].strip())


def build_screen_coordinate_map():
    """生成逻辑坐标到串口屏像素坐标的映射表。"""
    x_values = [85.5, 164.5, 243.5, 322.5, 401.5, 480.5, 559.5, 638.5, 717.5]
    y_values = [522.5, 444.2, 365.8, 287.5, 209.2, 130.8, 52.5]
    mapping = {}
    for a_idx, x_value in enumerate(x_values, 1):
        for b_idx, y_value in enumerate(y_values, 1):
            mapping[("A%d" % a_idx, "B%d" % b_idx)] = (x_value, y_value)
    return mapping


def screen_point(coord):
    """兼容 ('A1','B1') 和 ('A1',' B1') 两种坐标格式。"""
    return flc[(coord[0].strip(), coord[1].strip())]


def normalize_coord(coord):
    """统一坐标格式，去掉 A/B 两部分可能存在的空格。"""
    return coord[0].strip(), coord[1].strip()


def parse_forbidden_item(item):
    """把保存为字符串的禁飞区坐标还原为 ('A?', 'B?') 元组。"""
    if isinstance(item, tuple):
        return item[0].strip(), item[1].strip()
    parsed = ast.literal_eval(item)
    return parsed[0].strip(), parsed[1].strip()


def build_forbidden_frame(forbidden_area):
    """按协议构造禁飞区上行帧：AA A+0xA0 B+0xB0 ... FF。"""
    if len(forbidden_area) != 3:
        raise ValueError("禁飞区数量必须为 3 个")

    frame = [AIR_FRAME_HEADER]
    for a_str, b_str in forbidden_area:
        a_value = int(a_str.strip()[1:])
        b_value = int(b_str.strip()[1:])
        if not (1 <= a_value <= 9 and 1 <= b_value <= 7):
            raise ValueError("禁飞区坐标超出范围：%s, %s" % (a_str, b_str))
        frame.append(a_value + 0xA0)
        frame.append(b_value + 0xB0)
    frame.append(AIR_FRAME_TAIL)
    return bytes(frame)


def format_frame_hex(frame):
    """把二进制帧格式化成便于串口屏/日志查看的十六进制文本。"""
    return " ".join("%02X" % byte for byte in frame)


def send_forbidden_zones(forbidden_area):
    """把 3 个禁飞区按空地协同协议发送给树莓派/无人机端。"""
    forbidden_frame = build_forbidden_frame(forbidden_area)
    forbidden_frame_text = format_frame_hex(forbidden_frame)
    print("发送禁飞区上行帧：%s" % forbidden_frame_text)
    ser_air.write(forbidden_frame)
    send_to_screen('t12.txt="%s"' % forbidden_frame_text)


def clean_screen_command(recv):
    """把串口屏发来的原始字节整理成普通命令文本。"""
    return recv.decode("utf-8", errors="ignore").replace("\xff", "").replace("\r", "").replace("\n", "").strip()


def handle_repeat_forbidden_signal(forbidden_area):
    """收到串口屏 ag 信号时，重新发送禁飞区数据。"""
    if forbidden_area is None or ser2.in_waiting == 0:
        return False

    recv = ser2.read(ser2.in_waiting)
    clean_command = clean_screen_command(recv)
    if clean_command == "ag":
        print("收到 ag 信号，重新发送禁飞区数据")
        send_to_screen('t12.txt="开始重新发送"')
        send_forbidden_zones(forbidden_area)
        time.sleep(1)
        send_to_screen('t12.txt="重新发送完成"')
        return True

    print("收到串口屏信号：%s" % recv)
    return False


def draw_forbidden_circles(forbidden_area):
    """在串口屏上用红色圆圈标记 3 个禁飞区。"""
    for item in forbidden_area:
        try:
            x_value, y_value = screen_point(item)
            x_value = int(x_value)
            y_value = int(y_value)
            print("正在画禁飞区圆圈：%s -> (%d, %d)" % (item, x_value, y_value))
            send_to_screen("cirs %d,%d,20,63488" % (x_value, y_value))
            time.sleep(0.2)
        except Exception as exc:
            print("禁飞区画圆失败：%s，错误：%s" % (item, exc))

    send_forbidden_zones(forbidden_area)


def draw_path(vp, forbidden_area=None):
    """在串口屏上绘制带箭头的路径线段。"""
    print("等待串口屏发送 ok，准备绘制路径")
    while True:
        if ser2.inWaiting() == 0:
            time.sleep(0.05)
            continue

        recv = ser2.read(ser2.in_waiting)
        clean_command = clean_screen_command(recv)
        if clean_command == "ag":
            print("收到 ag 信号，重新发送禁飞区数据")
            send_forbidden_zones(forbidden_area)
            continue

        if clean_command != "ok":
            print("收到非 ok 信号：%s" % recv)
            continue

        print("收到开始绘制信号，开始画路径")
        for idx in range(len(vp) - 1):
            start_coord = vp[idx]
            end_coord = vp[idx + 1]

            try:
                n0 = int(screen_point(start_coord)[0])
                n1 = int(screen_point(start_coord)[1])
                n2 = int(screen_point(end_coord)[0])
                n3 = int(screen_point(end_coord)[1])
            except KeyError:
                print("坐标不在屏幕映射表中：%s 或 %s" % (start_coord, end_coord))
                continue

            color = 1040 + idx * 100
            print("线段 %d：%s -> %s，像素 (%d,%d) -> (%d,%d)" % (
                idx + 1,
                start_coord,
                end_coord,
                n0,
                n1,
                n2,
                n3,
            ))

            if n0 == n2:
                if n3 < n1:
                    for offset in DRAW_LINE_OFFSETS:
                        send_to_screen("line %d,%d,%d,%d,%d" % (n0 + offset, n1, n2 + offset, n3, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 + 10, n3 + offset + 10, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 - 10, n3 + offset + 10, color))
                else:
                    for offset in DRAW_LINE_OFFSETS:
                        send_to_screen("line %d,%d,%d,%d,%d" % (n0, n1 + offset, n2, n3 + offset, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 - 10, n3 + offset - 10, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 + 10, n3 + offset - 10, color))
            elif n1 == n3:
                if n2 < n0:
                    for offset in DRAW_LINE_OFFSETS:
                        send_to_screen("line %d,%d,%d,%d,%d" % (n0, n1 + offset, n2, n3 + offset, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 + 10, n3 + offset + 10, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 + 10, n3 + offset - 10, color))
                else:
                    for offset in DRAW_LINE_OFFSETS:
                        send_to_screen("line %d,%d,%d,%d,%d" % (n0 + offset, n1, n2 + offset, n3, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 - 10, n3 + offset - 10, color))
                        send_to_screen("line %d,%d,%d,%d,%d" % (n2, n3 + offset, n2 - 10, n3 + offset + 10, color))

            time.sleep(DRAW_LINE_DELAY)

        print("路径绘制完成")
        send_to_screen('t0.txt="1"')
        break


def receive_monitor_data(redbull, forbidden_area=None):
    """通过航点序号判断当前位置，并按预设动物坐标表显示动物信息。"""
    animal_totals = {"老虎": 0, "狼": 0, "猴子": 0, "孔雀": 0, "大象": 0}
    animal_screen_ids = {"猴子": 16, "大象": 18, "孔雀": 14, "老虎": 20, "狼": 22}

    total_waypoints = len(redbull)
    displayed_animal_coords = set()
    display_line = 1

    air_parser = AirProtocolFrameParser()
    print("开始监听航点序号，并按预设动物坐标显示结果")

    while True:
        time.sleep(0.05)
        handle_repeat_forbidden_signal(forbidden_area)

        if ser_air.in_waiting:
            data = ser_air.read(ser_air.in_waiting)
            air_parser.feed_data(data)

            while True:
                result = air_parser.parse_frame()
                if result is None:
                    break

                waypoint_idx, animal_cls, animal_cnt = result
                print("收到空地帧：idx=%d, cls=0x%02X, cnt=%d" % (waypoint_idx, animal_cls, animal_cnt))

                if waypoint_idx >= total_waypoints:
                    print("航点序号超出路径范围，已忽略：%d / %d" % (waypoint_idx, total_waypoints))
                    continue

                waypoint_coord = normalize_coord(redbull[waypoint_idx])
                current_coord_text = format_coord_display(waypoint_coord)
                send_to_screen('t31.txt="{}"'.format(current_coord_text))
                send_to_screen('t32.txt="航点{}:{}"'.format(waypoint_idx, current_coord_text))
                send_number_to_screen("n0", waypoint_idx)
                send_number_to_screen("n1", total_waypoints - 1)

                if waypoint_coord in flc:
                    x_value, y_value = screen_point(waypoint_coord)
                    send_number_to_screen("n2", int(x_value))
                    send_number_to_screen("n3", int(y_value))

                if waypoint_coord in PREDEFINED_ANIMALS and waypoint_coord not in displayed_animal_coords:
                    animal_name, count = PREDEFINED_ANIMALS[waypoint_coord]
                    displayed_animal_coords.add(waypoint_coord)
                    animal_totals[animal_name] += count

                    screen_id = animal_screen_ids[animal_name]
                    send_to_screen('page1.t%d.txt="%s"' % (screen_id, animal_totals[animal_name]))

                    record_text = "%s,%s:%d" % (current_coord_text, animal_name, count)
                    display_line += 1
                    command = 'page0.t%d.txt="%s"' % (display_line, record_text)
                    print("到达预设动物坐标：%s" % record_text)
                    send_to_screen(command)

                if waypoint_idx >= total_waypoints - 1:
                    print("所有航点已完成")
                    send_to_screen('t31.txt="飞行完成!"')
                    return

        time.sleep(0.05)


def main():
    global ser_air, ser2, flc

    # ========== 串口初始化 ==========
    ser2 = serial.Serial("/dev/my_screen", 9600)  # 串口屏
    ser_air = serial.Serial(AIR_PORT, AIR_BAUDRATE, timeout=0.05)  # 单个空地通信蓝牙串口

    flc = build_screen_coordinate_map()

    # ========== GPIO 初始化 ==========
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT, initial=GPIO.HIGH)

    send_to_screen('t12.txt="请输入禁飞区"')
    print("Raspberry pi is ready")
    time.sleep(2)

    # ========== 接收 3 个禁飞区 ==========
    forbidden_input = []
    print("准备接收第 1 个禁飞区坐标")
    while len(forbidden_input) < 3:
        if ser2.inWaiting() == 0:
            time.sleep(0.1)
            continue

        time.sleep(0.5)
        recv = ser2.read(ser2.in_waiting)
        clean_command = recv.strip().decode("utf-8", errors="ignore")
        clean_data = clean_command.replace("/n", "").replace("\n", "").strip()

        if "," not in clean_data:
            continue

        parts = [part.strip() for part in clean_data.split(",")]
        if len(parts) != 2:
            continue

        formatted = "('%s', '%s')" % (parts[0], parts[1])
        if formatted in forbidden_input:
            continue

        forbidden_input.append(formatted)
        print("已录入第 %d 个禁飞区：%s" % (len(forbidden_input), formatted))
        send_to_screen('t12.txt="%s"' % clean_data)

        if len(forbidden_input) < 3:
            print("准备接收第 %d 个禁飞区坐标" % (len(forbidden_input) + 1))

    forbidden_area = [parse_forbidden_item(item) for item in forbidden_input]
    print("禁飞区坐标：%s" % forbidden_area)
    draw_forbidden_circles(forbidden_area)

    print("禁飞区选择完成")
    GPIO.output(18, GPIO.LOW)
    GPIO.cleanup()

    # ========== 路径规划 ==========
    print("开始路径规划")
    generate_flight_files(forbidden_area)
    time.sleep(5)

    send_to_screen('t12.txt="开始画路径"')

    # ========== 读取简化路径并绘制 ==========
    vp = parse_path_file("simplified_path.txt")
    print("简化路径点：%s" % vp)
    draw_path(vp, forbidden_area)

    send_to_screen('t12.txt="开始接收数据"')

    # ========== 接收航点和动物数据 ==========
    redbull = parse_path_file("flight_path.txt")
    print("完整航点数量：%d" % len(redbull))
    receive_monitor_data(redbull, forbidden_area)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("程序被用户中断，正在关闭串口")
        for serial_obj_name in ("ser_air", "ser2"):
            serial_obj = globals().get(serial_obj_name)
            if serial_obj is not None and serial_obj.is_open:
                serial_obj.close()
