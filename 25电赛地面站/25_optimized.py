# -*- coding: utf-8 -*-
"""
Ground Station Main Program
Function: Drone Flight Path Planning and Animal Detection System
"""

import RPi.GPIO as GPIO
import serial
import time
import re
import path_generator
from simple import *

# ============================================
# Configuration
# ============================================
SERIAL_BAUDRATE = 9600
SCREEN_PORT = '/dev/ttyUSB0'
BLUETOOTH_WILDLIFE_PORT = '/dev/my-bluetooth'
BLUETOOTH_POSITION_PORT = '/dev/my-bluetooth2'

SCREEN_END_BYTES = bytes.fromhex('ff ff ff')

CIRCLE_RADIUS = 20
CIRCLE_COLOR = 63488
LINE_COLOR_BASE = 1040

START_LOCATION = "A9, B1"
FLIGHT_PATH_FILE = "flight_path.txt"
SIMPLIFIED_PATH_FILE = "simplified_path.txt"
OUTPUT_PATH_FILE = "flight_path.txt"
ROUTER_FILE = "router.txt"

# Coordinate mapping table (A1-A9, B1-B7)
COORDINATE_MAP = {
    ('A1', ' B1'): (85.5, 522.5), ('A2', ' B1'): (164.5, 522.5),
    ('A3', ' B1'): (243.5, 522.5), ('A4', ' B1'): (322.5, 522.5),
    ('A5', ' B1'): (401.5, 522.5), ('A6', ' B1'): (480.5, 522.5),
    ('A7', ' B1'): (559.5, 522.5), ('A8', ' B1'): (638.5, 522.5),
    ('A9', ' B1'): (717.5, 522.5), ('A1', ' B2'): (85.5, 444.2),
    ('A2', ' B2'): (164.5, 444.2), ('A3', ' B2'): (243.5, 365.8),
    ('A4', ' B2'): (322.5, 444.2), ('A5', ' B2'): (401.5, 444.2),
    ('A6', ' B2'): (480.5, 444.2), ('A7', ' B2'): (559.5, 444.2),
    ('A8', ' B2'): (638.5, 444.2), ('A9', ' B2'): (717.5, 444.2),
    ('A1', ' B3'): (85.5, 365.8), ('A2', ' B3'): (164.5, 365.8),
    ('A3', ' B3'): (243.5, 365.8), ('A4', ' B3'): (322.5, 365.8),
    ('A5', ' B3'): (401.5, 365.8), ('A6', ' B3'): (480.5, 365.8),
    ('A7', ' B3'): (559.5, 365.8), ('A8', ' B3'): (638.5, 365.8),
    ('A9', ' B3'): (717.5, 365.8), ('A1', ' B4'): (85.5, 287.5),
    ('A2', ' B4'): (164.5, 287.5), ('A3', ' B4'): (243.5, 287.5),
    ('A4', ' B4'): (322.5, 287.5), ('A5', ' B4'): (401.5, 287.5),
    ('A6', ' B4'): (480.5, 287.5), ('A7', ' B4'): (559.5, 287.5),
    ('A8', ' B4'): (638.5, 287.5), ('A9', ' B4'): (717.5, 287.5),
    ('A1', ' B5'): (85.5, 209.2), ('A2', ' B5'): (164.5, 209.2),
    ('A3', ' B5'): (243.5, 209.2), ('A4', ' B5'): (322.5, 209.2),
    ('A5', ' B5'): (401.5, 209.2), ('A6', ' B5'): (480.5, 209.2),
    ('A7', ' B5'): (559.5, 209.2), ('A8', ' B5'): (638.5, 209.2),
    ('A9', ' B5'): (717.5, 209.2), ('A1', ' B6'): (85.5, 130.8),
    ('A2', ' B6'): (164.5, 130.8), ('A3', ' B6'): (243.5, 130.8),
    ('A4', ' B6'): (322.5, 130.8), ('A5', ' B6'): (401.5, 130.8),
    ('A6', ' B6'): (480.5, 130.8), ('A7', ' B6'): (559.5, 130.8),
    ('A8', ' B6'): (638.5, 130.8), ('A9', ' B6'): (717.5, 130.8),
    ('A1', ' B7'): (85.5, 52.5), ('A2', ' B7'): (164.5, 52.5),
    ('A3', ' B7'): (243.5, 52.5), ('A4', ' B7'): (322.5, 52.5),
    ('A5', ' B7'): (401.5, 52.5), ('A6', ' B7'): (480.5, 52.5),
    ('A7', ' B7'): (559.5, 52.5), ('A8', ' B7'): (638.5, 52.5),
    ('A9', ' B7'): (717.5, 52.5)
}

ANIMAL_NAMES = {'6': 0, '8': 0, '4': 0, '0': 0, '2': 0}
ANIMAL_TO_SCREEN_ID = {'6': 0, '8': 0, '4': 0, '0': 0, '2': 0}
ANIMAL_DISPLAY_NAMES = {'6': 'Monkey', '8': 'Elephant', '4': 'Peacock', '0': 'Tiger', '2': 'Wolf'}
SCREEN_ID_MAP = {'Monkey': 16, 'Elephant': 18, 'Peacock': 14, 'Tiger': 20, 'Wolf': 22}


# ============================================
# Helper Functions
# ============================================
def send_screen_cmd(serial_conn, cmd):
    """Send command to screen with standard ending"""
    serial_conn.write(cmd.encode("GB2312"))
    serial_conn.write(SCREEN_END_BYTES)
    serial_conn.flushInput()


def parse_coordinate(raw_data):
    """Parse coordinate string like 'A1, B7' into tuple ('A1', 'B7')"""
    clean_data = raw_data.replace('/n', '').replace('\n', '').strip()
    if ',' in clean_data:
        parts = clean_data.split(',')
        if len(parts) == 2:
            coord_a = parts[0].strip()
            coord_b = parts[1].strip()
            return (coord_a, coord_b)
    return None


def lookup_physical_coordinate(logic_coord):
    """Look up physical coordinate from mapping table"""
    if isinstance(logic_coord, str):
        coord_tuple = parse_coordinate(logic_coord)
        if coord_tuple:
            lookup_key = (coord_tuple[0], ' ' + coord_tuple[1])
            return COORDINATE_MAP.get(lookup_key)
        return None
    return COORDINATE_MAP.get(logic_coord)


def extract_coordinates_from_file(file_path):
    """Extract coordinates from flight path file"""
    coordinates = []
    with open(file_path, 'r') as file:
        for line in file:
            coord_str = line.split(': ', 1)[-1].strip()
            if coord_str.startswith('(') and coord_str.endswith(')'):
                coord = coord_str[1:-1].split(', ')
                if len(coord) == 2:
                    coordinates.append((coord[0], coord[1]))
    return coordinates


def draw_circle(serial_conn, x, y, color=CIRCLE_COLOR):
    """Draw circle at specified physical coordinate"""
    draw_cmd = f'cirs {int(x)},{int(y)},{CIRCLE_RADIUS},{color}'
    send_screen_cmd(serial_conn, draw_cmd)
    time.sleep(0.2)


def draw_path_line(serial_conn, start_coord, end_coord, color_id, coord_map):
    """Draw a path line between two coordinates"""
    if start_coord in coord_map and end_coord in coord_map:
        n0, n1 = int(coord_map[start_coord][0]), int(coord_map[start_coord][1])
        n2, n3 = int(coord_map[end_coord][0]), int(coord_map[end_coord][1])

        if n0 == n2:
            for j in range(-3, 3):
                cmd1 = f'line {n0 + j},{n1},{n2 + j},{n3},{color_id}'
                cmd2 = f'line {n2},{n3 + j},{n2 + 10},{n3 + j + 10},{color_id}'
                cmd3 = f'line {n2},{n3 + j},{n2 - 10},{n3 + j + 10},{color_id}'
                send_screen_cmd(serial_conn, cmd1)
                send_screen_cmd(serial_conn, cmd2)
                send_screen_cmd(serial_conn, cmd3)
        elif n1 == n3:
            for j in range(-3, 3):
                cmd1 = f'line {n0},{n1 + j},{n2},{n3 + j},{color_id}'
                cmd2 = f'line {n2},{n3 + j},{n2 + 10},{n3 + j + 10},{color_id}'
                cmd3 = f'line {n2},{n3 + j},{n2 - 10},{n3 + j - 10},{color_id}'
                send_screen_cmd(serial_conn, cmd1)
                send_screen_cmd(serial_conn, cmd2)
                send_screen_cmd(serial_conn, cmd3)


# ============================================
# Main Program
# ============================================
def main():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(18, GPIO.OUT, initial=GPIO.HIGH)

    ser2 = serial.Serial(SCREEN_PORT, SERIAL_BAUDRATE)
    print("Raspberry pi is ready")

    # ----------------------------------------
    # Phase 1: Collect forbidden zone coordinates
    # ----------------------------------------
    send_screen_cmd(ser2, 't12.txt="Please input forbidden zone"')

    forbidden_zones = []
    print("Ready to receive 1st coordinate...")

    while len(forbidden_zones) < 3:
        if ser2.inWaiting() != 0:
            time.sleep(0.5)
            recv = ser2.read(ser2.in_waiting)
            clean_data = recv.strip().decode('utf-8', errors='ignore')
            clean_data = clean_data.replace('/n', '').replace('\n', '').strip()

            coord = parse_coordinate(clean_data)
            if coord and coord not in forbidden_zones:
                forbidden_zones.append(coord)
                print(f"Recorded point {len(forbidden_zones)}: {coord}")
                send_screen_cmd(ser2, f't12.txt="{clean_data}"')
                if len(forbidden_zones) < 3:
                    print(f"Ready to receive {len(forbidden_zones) + 1}th coordinate...")

    print("\nAll 3 points collected, drawing circles...")

    for coord in forbidden_zones:
        physical_coord = lookup_physical_coordinate(coord)
        if physical_coord:
            print(f"Drawing circle at logic {coord} -> physical {physical_coord}")
            draw_circle(ser2, physical_coord[0], physical_coord[1])
        else:
            print(f"Coordinate not found in map: {coord}")

    print("\nAll circle commands executed.")
    print(forbidden_zones)

    # ----------------------------------------
    # Phase 2: Wait for fly command
    # ----------------------------------------
    while True:
        if ser2.inWaiting() != 0:
            recv = ser2.read(ser2.in_waiting)
            if recv == b'ag':
                fly_cmd = 'FLY_CMD:%s,%s;%s,%s;%s,%s/np' % (
                    forbidden_zones[0][0], forbidden_zones[0][1],
                    forbidden_zones[1][0], forbidden_zones[1][1],
                    forbidden_zones[2][0], forbidden_zones[2][1]
                )
                print(fly_cmd)
                send_screen_cmd(ser2, f't12.txt="{fly_cmd}"')
            elif recv == b'ok':
                break

    forbidden_area_str = '[("%s","%s"),("%s","%s"),("%s","%s")]' % (
        forbidden_zones[0][0], forbidden_zones[0][1],
        forbidden_zones[1][0], forbidden_zones[1][1],
        forbidden_zones[2][0], forbidden_zones[2][1]
    )
    print(forbidden_area_str)
    print('Finished')

    GPIO.output(18, GPIO.LOW)
    GPIO.cleanup()
    forbidden_area = eval(forbidden_area_str)

    # ----------------------------------------
    # Phase 3: Generate flight path
    # ----------------------------------------
    print('Generating flight plan...')
    path_generator.generate_flight_plan(
        start_point_hr=START_LOCATION,
        forbidden_zones_tuples=forbidden_area
    )

    result = simplify_path_from_file(FLIGHT_PATH_FILE, SIMPLIFIED_PATH_FILE)
    print('Flight plan generated')

    # ----------------------------------------
    # Phase 4: Draw flight path on screen
    # ----------------------------------------
    waypoints = extract_coordinates_from_file(SIMPLIFIED_PATH_FILE)
    print(waypoints)

    start_physical = lookup_physical_coordinate(('A9', ' B1'))
    current_pos = start_physical
    line_counter = 0

    print("Ready to draw path")

    while True:
        if ser2.inWaiting() != 0:
            recv = ser2.read(ser2.in_waiting)
            if recv == b'ok':
                print("Start signal received, drawing path...")

                for i in range(len(waypoints) - 1):
                    line_counter += 1
                    print(f"Line {line_counter}: {waypoints[i]} -> {waypoints[i + 1]}")
                    color_id = LINE_COLOR_BASE + i * 100
                    draw_path_line(ser2, waypoints[i], waypoints[i + 1], color_id, COORDINATE_MAP)
                    time.sleep(0.1)

                print("Path drawing completed!")
                send_screen_cmd(ser2, 't0.txt="1"')
                break

    # ----------------------------------------
    # Phase 5: Receive animal detection data
    # ----------------------------------------
    send_screen_cmd(ser2, 't12.txt="Start receiving data"')

    animal_counts = ANIMAL_NAMES.copy()
    waypoint_data = None
    waypoint_display_index = 0
    total_waypoints = len(waypoints)
    eocat = {}
    current_waypoint_info = ''

    while True:
        time.sleep(0.1)

        # Wildlife bluetooth data
        if ser3.inWaiting() != 0:
            response3 = ser3.readline()
            if response3:
                clean_command = response3.strip().decode('utf-8', errors='ignore')
                print(clean_command)
                if clean_command.endswith('/p') and len(eocat) != 0:
                    if eocat.get(current_waypoint_info, '') == '':
                        temperature = clean_command[:-2]
                        faze = temperature.split(",")
                        processed_animals = []

                        for animal_data in faze:
                            parts = animal_data.split(':')
                            if len(parts) > 2:
                                parts = [parts[0], parts[1][0]]
                            else:
                                parts = [parts[0], '1']

                            animal_name = parts[0]
                            if parts[-1].isdigit():
                                animal_count = int(parts[-1])
                            else:
                                if len(parts[-1]) > 1 and parts[-1][0].isdigit():
                                    animal_count = int(parts[-1][0])
                                else:
                                    animal_count = 0

                            if animal_name in animal_counts:
                                if animal_name not in processed_animals:
                                    if animal_count != 0:
                                        processed_animals.append(animal_name)
                                        animal_counts[animal_name] += animal_count
                                        screen_id = SCREEN_ID_MAP.get(ANIMAL_DISPLAY_NAMES.get(animal_name, ''), 0)
                                        send_screen_cmd(ser2, f'page1.t{screen_id}.txt="{animal_counts[animal_name]}"')
                                        current_waypoint_info += f',{ANIMAL_DISPLAY_NAMES.get(animal_name, animal_name)}:{animal_count}'
                                        print(f'g2={current_waypoint_info}')

                        send_screen_cmd(ser2, f'page0.g{waypoint_display_index}.txt="{current_waypoint_info}"')
                        eocat[current_waypoint_info] = 'ok'
                        current_waypoint_info = ''
                        waypoint_display_index += 1

        ser3.flushInput()
        time.sleep(0.1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        if 'ser1' in dir():
            ser1.close()
        if 'ser2' in dir():
            ser2.close()
        if 'ser3' in dir():
            ser3.close()
        print("Program terminated")