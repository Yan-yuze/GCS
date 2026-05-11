# ESP8266 esp-link 串口转 Wi-Fi 部署总结

本文档记录 ESP8266 使用 `esp-link` 固件实现串口转 Wi-Fi 的完整流程，适用于以下链路：

```text
树莓派 Python <--Wi-Fi/TCP--> ESP8266 <--串口--> 香橙派/下位机
```

ESP8266 在这个方案中不是替代树莓派的主控，而是作为“无线串口桥”。树莓派通过 TCP socket 连接 ESP8266，ESP8266 再通过 UART 串口与香橙派或其他下位机通信。

## 1. 方案说明

### 1.1 ESP8266 的作用

ESP8266 负责把 Wi-Fi TCP 数据和 UART 串口数据互相转发：

```text
TCP 数据 -> ESP8266 -> 串口 TX
串口 RX -> ESP8266 -> TCP 数据
```

使用 `esp-link` 固件后，默认透明串口桥端口为：

```text
TCP 端口：23
```

### 1.2 两种网络模式

#### STA 模式

ESP8266 连接已有路由器或热点，例如当前测试时获得：

```text
ESP8266 IP: 192.168.2.103
TCP Port: 23
```

这种模式适合实验室调试，但比赛现场如果没有路由器，IP 可能不可用。

#### AP 模式

ESP8266 自己开热点，树莓派连接 ESP8266 热点：

```text
ESP8266 AP IP: 192.168.4.1
TCP Port: 23
```

比赛现场更推荐 AP 模式，因为不需要额外路由器、手机或电脑热点。

推荐比赛链路：

```text
树莓派连接 ESP8266 热点
树莓派 Python -> 192.168.4.1:23
ESP8266 串口 -> 香橙派
```

## 2. 烧录准备

### 2.1 安装 esptool

Windows PowerShell 中安装：

```powershell
python -m pip install esptool
```

Windows 下建议使用：

```powershell
python -m esptool
```

不要使用：

```powershell
esptool.py
```

如果出现 `esptool.py 无法识别`，说明该命令没有加入 PATH，不影响使用 `python -m esptool`。

### 2.2 检测 ESP8266

进入烧录模式后执行：

```powershell
python -m esptool --chip esp8266 --port COM25 flash-id
```

本次测试识别结果：

```text
Chip type: ESP8266EX
Detected flash size: 4MB
```

因此后续按 4MB Flash 烧录。

## 3. 烧录时接线

ESP8266 烧录时必须进入下载模式。

### 3.1 USB-TTL 与 ESP8266 接线

```text
USB-TTL TX  -> ESP8266 RX / GPIO3
USB-TTL RX  -> ESP8266 TX / GPIO1
USB-TTL GND -> ESP8266 GND
USB-TTL 3.3V -> ESP8266 VCC
```

注意：

- ESP8266 是 3.3V 供电和 3.3V 串口电平
- 不要给 VCC 接 5V
- 不要用 5V TTL 直接接 ESP8266 RX
- GND 必须共地

### 3.2 烧录模式启动脚

烧录时：

```text
GPIO0 -> GND
GPIO2 -> 3.3V
GPIO15 -> GND    如果模块引出了 GPIO15
EN/CH_PD -> 3.3V
RST -> 上拉到 3.3V，或按键复位
```

进入烧录模式的方法：

1. 断电
2. GPIO0 接 GND
3. 重新上电或按 RST
4. 执行 esptool 命令

## 4. 烧录 esp-link 固件

### 4.1 下载固件

推荐使用稳定版：

```text
esp-link v3.0.14
```

下载地址：

```text
https://github.com/jeelabs/esp-link/releases/tag/V3.0.14
```

解压后进入包含以下文件的目录：

```text
boot_v1.6.bin
user1.bin
esp_init_data_default.bin
blank.bin
```

如果 boot 文件名不同，例如 `boot_v1.7.bin`，烧录命令中需要同步修改文件名。

### 4.2 4MB Flash 烧录命令

本次 ESP8266 检测为 4MB Flash，因此使用：

```powershell
python -m esptool --chip esp8266 --port COM25 --baud 115200 write-flash --flash-size 4MB --flash-mode dio --flash-freq 40m 0x00000 boot_v1.6.bin 0x01000 user1.bin 0x3FC000 esp_init_data_default.bin 0x3FE000 blank.bin
```

烧录成功时会看到多次：

```text
Hash of data verified.
```

最后类似：

```text
Hard resetting via RTS pin...
```

表示烧录完成。

## 5. 正常运行时接线

烧录完成后，必须退出烧录模式。

### 5.1 正常启动脚状态

```text
GPIO0 -> 不接 GND，最好上拉到 3.3V
GPIO2 -> 3.3V
GPIO15 -> GND    如果模块引出了 GPIO15
EN/CH_PD -> 3.3V
VCC -> 3.3V
GND -> GND
```

正常运行时，GPIO0 不能接 GND，否则 ESP8266 会再次进入烧录模式，不会运行 esp-link，也不会开热点。

### 5.2 ESP8266 与香橙派串口接线

运行时目标链路：

```text
树莓派 Python <--Wi-Fi--> ESP8266 <--UART--> 香橙派
```

ESP8266 与香橙派 UART 接线：

```text
ESP8266 TX / GPIO1 -> 香橙派 RX
ESP8266 RX / GPIO3 -> 香橙派 TX
ESP8266 GND        -> 香橙派 GND
```

注意：

- TX 和 RX 要交叉连接
- 必须共地
- 香橙派 UART 电平通常为 3.3V，适合直接连接 ESP8266
- 确认香橙派串口没有被系统控制台占用

## 6. esp-link 初次配置

### 6.1 首次启动

正常启动后，ESP8266 会开热点，名称通常类似：

```text
ESP_XXXXXX
ai-thinker-XXXXXX
```

连接该热点后访问：

```text
http://192.168.4.1
```

### 6.2 STA 模式配置

在 esp-link 网页中进入 WiFi 页面：

1. 扫描 Wi-Fi
2. 选择路由器 Wi-Fi
3. 输入密码
4. Connect / Save

连接成功后，ESP8266 会获得路由器分配的 IP，例如：

```text
http://192.168.2.103
```

此时 TCP 串口桥为：

```text
192.168.2.103:23
```

### 6.3 AP 模式配置

比赛现场推荐 AP 模式。

从当前 STA 页面进入：

```text
http://192.168.2.103
```

在 WiFi 页面找到：

```text
Switch to AP mode
```

切换后 ESP8266 会断开路由器连接，并重新开启自己的热点。

连接 ESP8266 热点后访问：

```text
http://192.168.4.1
```

当前已设置好的 Soft-AP 参数：

```text
Soft-AP SSID: ESP_LINK
Soft-AP Password: 12345678
Soft-AP Auth Mode: WPA_WPA2_PSK
Soft-AP Max Connections: 4
Soft-AP Beacon Interval: 100
Soft-AP SSID hidden: disabled
```

点击 `Change Soft-AP settings!` 保存后，如果电脑或树莓派 Wi-Fi 断开，属于正常现象。重新连接 `ESP_LINK` 热点即可。

AP 模式下，树莓派 Python 程序固定连接：

```text
192.168.4.1:23
```

## 7. 串口桥测试

### 7.1 网络调试助手测试

网络调试助手配置：

```text
协议：TCP Client
远程地址：ESP8266 IP
远程端口：23
```

STA 模式示例：

```text
192.168.2.103:23
```

AP 模式示例：

```text
192.168.4.1:23
```

### 7.2 串口助手配置

串口助手配置需要和 esp-link 网页中的串口波特率一致：

```text
波特率：9600 或 115200，必须两边一致
数据位：8
校验位：None
停止位：1
流控：None
```

### 7.3 最小测试

网络调试助手发送 ASCII：

```text
A
```

串口助手 HEX 显示应收到：

```text
41
```

串口助手发送 ASCII：

```text
B
```

网络调试助手 HEX 显示应收到：

```text
42
```

如果使用 HEX 模式发送：

```text
FE
```

另一端收到 `FE` 是正常的，表示发送了一个原始字节 `0xFE`。

## 8. 树莓派 Python 测试代码

### 8.1 TCP ASCII 测试

AP 模式下推荐连接：

```python
ESP_IP = "192.168.4.1"
ESP_PORT = 23
```

测试脚本：

```python
import socket
import time

ESP_IP = "192.168.4.1"
ESP_PORT = 23

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(2)

print("Connecting...")
sock.connect((ESP_IP, ESP_PORT))
print("Connected to ESP8266")

try:
    while True:
        msg = input("Send> ")
        if msg.lower() in ("q", "quit", "exit"):
            break

        sock.sendall(msg.encode("ascii"))
        time.sleep(0.1)

        try:
            data = sock.recv(1024)
            if data:
                print("Recv ASCII:", data.decode("ascii", errors="replace"))
                print("Recv HEX:", data.hex(" "))
            else:
                print("Connection closed")
                break
        except socket.timeout:
            print("No response")

finally:
    sock.close()
    print("Closed")
```

运行：

```bash
python3 test_esp8266_tcp.py
```

### 8.2 HEX 帧测试

如果需要发送二进制协议帧：

```python
import socket

ESP_IP = "192.168.4.1"
ESP_PORT = 23

with socket.create_connection((ESP_IP, ESP_PORT), timeout=3) as sock:
    sock.settimeout(2)

    frame = bytes.fromhex("AA 55 00 01 00 10 11 0D")
    sock.sendall(frame)
    print("Sent:", frame.hex(" "))

    try:
        data = sock.recv(1024)
        print("Recv:", data.hex(" "))
    except socket.timeout:
        print("No response")
```

## 9. 将原串口代码改为 socket

原来树莓派代码中可能使用：

```python
import serial

ser1 = serial.Serial("/dev/ttyUSB1", 9600, timeout=0.5)
data = ser1.read(ser1.in_waiting)
ser1.write(b"hello")
```

改成 TCP socket：

```python
import socket

sock1 = socket.create_connection(("192.168.4.1", 23), timeout=3)
sock1.settimeout(0.5)

data = sock1.recv(1024)
sock1.sendall(b"hello")
```

如果原来的解析器接收的是 `bytes`，例如：

```python
bt_parser.feed_data(data)
animal_parser.feed_data(data)
```

则解析器逻辑一般不需要大改，只需要把数据来源从 `serial.read()` 换成 `socket.recv()`。

建议封装一个简单类，让 socket 的用法尽量接近串口：

```python
import socket

class TcpSerialBridge:
    def __init__(self, host, port=23, timeout=0.5):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=3)
        self.sock.settimeout(self.timeout)

    def read(self, size=1024):
        try:
            return self.sock.recv(size)
        except socket.timeout:
            return b""

    def write(self, data):
        self.sock.sendall(data)

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
```

使用：

```python
ser_wifi = TcpSerialBridge("192.168.4.1", 23, timeout=0.5)
ser_wifi.connect()

data = ser_wifi.read()
if data:
    bt_parser.feed_data(data)

ser_wifi.write(b"hello")
```

## 10. 常见问题

### 10.1 `esptool.py` 无法识别

现象：

```text
esptool.py : 无法将“esptool.py”项识别为 cmdlet
```

解决：

```powershell
python -m esptool --chip esp8266 --port COM25 flash-id
```

### 10.2 COM 口被占用

现象：

```text
Could not open COM25, the port is busy or doesn't exist.
PermissionError(13, '拒绝访问。')
```

解决：

- 关闭串口助手
- 关闭 Arduino IDE 串口监视器
- 关闭 Putty、MobaXterm 等串口软件
- 拔插 USB-TTL
- 重新确认 COM 号

### 10.3 No serial data received

现象：

```text
Failed to connect to ESP8266: No serial data received.
```

常见原因：

- GPIO0 没有接 GND，未进入烧录模式
- TX/RX 接反或接触不良
- EN/CH_PD 没有接 3.3V
- GPIO2/GPIO15 启动电平错误
- 供电不足

### 10.4 找不到 esp-link 热点

先确认正常运行接线：

```text
GPIO0 不接 GND
GPIO2 -> 3.3V
GPIO15 -> GND
EN/CH_PD -> 3.3V
```

串口日志中如果看到：

```text
Wifi init, mode=AP
dhcp server start:(ip:192.168.4.1,mask:255.255.255.0,gw:192.168.4.1)
** esp-link ... ready
```

说明 AP 已经启动。

热点名称可能不是 `esp-link`，而是：

```text
ESP_XXXXXX
ai-thinker-XXXXXX
```

### 10.5 串口乱码

ESP8266 上电初期可能输出 ROM 启动日志，常见波特率为：

```text
74880
```

esp-link 后续日志常见为：

```text
115200
```

如果只是上电最开始短暂乱码，但后面出现 esp-link 日志，一般不影响使用。

### 10.6 网络能连但串口数据异常

检查：

- esp-link 网页中的串口波特率
- 串口助手波特率
- 数据位 8
- 校验 None
- 停止位 1
- 流控 None
- TX/RX 是否交叉
- GND 是否共地

## 11. 比赛部署建议

推荐部署模式：

```text
ESP8266 AP 模式
树莓派自动连接 ESP8266 热点
树莓派 Python 固定连接 192.168.4.1:23
ESP8266 串口连接香橙派
```

不要依赖现场路由器或手机热点。

Python 中建议把 IP 和端口放在配置区：

```python
ESP_IP = "192.168.4.1"
ESP_PORT = 23
```

如果临时改回路由器 STA 模式，只需要改为路由器分配的 IP：

```python
ESP_IP = "192.168.2.103"
ESP_PORT = 23
```

## 12. Wi-Fi 方案与蓝牙方案对比

### Wi-Fi / ESP8266 优点

- 速度更高
- TCP 传输更适合可靠数据流
- Python socket 调试方便
- ESP8266 可自建 AP，不需要额外路由器
- 后续可以扩展网页配置、UDP、多个客户端监听

### Wi-Fi / ESP8266 缺点

- 功耗高于蓝牙
- 对 3.3V 供电要求更高
- 需要处理 Wi-Fi 连接和 TCP 重连
- 如果比赛规则禁止 Wi-Fi，则不能使用

### 蓝牙方案优点

- 点对点，结构简单
- 功耗较低
- 不需要 IP 和端口配置

### 蓝牙方案缺点

- 速率和稳定性通常不如 Wi-Fi
- 配对和串口映射有时麻烦
- 距离通常更短
- 程序调试不如 TCP socket 直观

## 13. 关键参数记录

本次测试参数：

```text
ESP8266 芯片：ESP8266EX
Flash：4MB
测试 STA IP：192.168.2.103
AP 默认 IP：192.168.4.1
TCP 串口桥端口：23
固件：esp-link v3.0.14-g963ffbb
烧录串口：COM25
```

后续正式比赛建议使用：

```text
ESP8266 模式：AP-only
树莓派连接热点：ESP_LINK
热点密码：12345678
热点加密：WPA_WPA2_PSK
Python 连接地址：192.168.4.1:23
```
