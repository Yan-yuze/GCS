# -*- coding: utf-8 -*-
"""
DL-20 Bluetooth serial link tester for the Orange Pi side.

Wiring under test:
    Serial assistant <-> DL-20 <~~ Bluetooth ~~> DL-20 <-> Orange Pi Python

Protocol extracted from the 5.14 drone/ground-station code:
    Ground -> Orange Pi, forbidden zones, 8 bytes:
        AA A? B? A? B? A? B? FF
        Example: AA A1 B1 A2 B2 A3 B3 FF means
                 (A1,B1), (A2,B2), (A3,B3)

    Orange Pi -> Ground, progress/result, 5 bytes:
        AA idx cls cnt FF
        cls: 00 tiger, 01 wolf, 02 monkey, 03 peacock, 04 elephant, FF no animal
        cnt: 0 means flying/progress only, >=1 means detection result is valid

Examples:
    python dl20_airlink_test.py --port /dev/ttyS7 --baud 115200 --mode opi
    python dl20_airlink_test.py --port /dev/ttyUSB0 --baud 115200 --mode rx
    python dl20_airlink_test.py --port COM3 --baud 115200 --mode tx-progress
"""

import argparse
import time

import serial


HEADER = 0xAA
TAIL = 0xFF
FORBIDDEN_FRAME_LEN = 8
STATUS_FRAME_LEN = 5

ANIMAL_NAMES = {
    0x00: "tiger",
    0x01: "wolf",
    0x02: "monkey",
    0x03: "peacock",
    0x04: "elephant",
    0xFF: "none",
}


def hex_bytes(data):
    return " ".join("{:02X}".format(byte) for byte in data)


def build_status_frame(waypoint_idx, animal_cls=0xFF, animal_cnt=0):
    """Build the Orange Pi -> ground station 5-byte frame: AA idx cls cnt FF."""
    for name, value in (
        ("waypoint_idx", waypoint_idx),
        ("animal_cls", animal_cls),
        ("animal_cnt", animal_cnt),
    ):
        if not 0 <= value <= 0xFF:
            raise ValueError("{} must be in 0..255".format(name))
    return bytes([HEADER, waypoint_idx, animal_cls, animal_cnt, TAIL])


def build_forbidden_frame(zones):
    """Build an 8-byte forbidden-zone frame from [('A1', 'B1'), ...]."""
    if len(zones) != 3:
        raise ValueError("exactly 3 forbidden zones are required")

    frame = [HEADER]
    for a_text, b_text in zones:
        a_value = int(a_text.upper().lstrip("A"))
        b_value = int(b_text.upper().lstrip("B"))
        if not 1 <= a_value <= 9:
            raise ValueError("A coordinate out of range: {}".format(a_text))
        if not 1 <= b_value <= 7:
            raise ValueError("B coordinate out of range: {}".format(b_text))
        frame.append(0xA0 + a_value)
        frame.append(0xB0 + b_value)
    frame.append(TAIL)
    return bytes(frame)


def decode_forbidden_frame(frame):
    """Decode AA A? B? A? B? A? B? FF into [('A1', 'B1'), ...]."""
    if len(frame) != FORBIDDEN_FRAME_LEN:
        raise ValueError("forbidden frame length must be 8 bytes")
    if frame[0] != HEADER or frame[-1] != TAIL:
        raise ValueError("forbidden frame header/tail mismatch")

    zones = []
    for offset in (1, 3, 5):
        a_raw = frame[offset]
        b_raw = frame[offset + 1]
        a_value = a_raw - 0xA0
        b_value = b_raw - 0xB0
        if not 1 <= a_value <= 9 or not 1 <= b_value <= 7:
            raise ValueError("invalid zone bytes: {:02X} {:02X}".format(a_raw, b_raw))
        zones.append(("A{}".format(a_value), "B{}".format(b_value)))
    return zones


def decode_status_frame(frame):
    """Decode AA idx cls cnt FF into a readable tuple."""
    if len(frame) != STATUS_FRAME_LEN:
        raise ValueError("status frame length must be 5 bytes")
    if frame[0] != HEADER or frame[-1] != TAIL:
        raise ValueError("status frame header/tail mismatch")
    waypoint_idx, animal_cls, animal_cnt = frame[1], frame[2], frame[3]
    return waypoint_idx, animal_cls, animal_cnt


class FrameParser:
    """Small resyncing parser for either 8-byte forbidden frames or 5-byte status frames."""

    def __init__(self, frame_len, decoder):
        self.frame_len = frame_len
        self.decoder = decoder
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)

    def parse_all(self):
        results = []
        while True:
            header_idx = self.buffer.find(bytes([HEADER]))
            if header_idx < 0:
                self.buffer.clear()
                return results

            if len(self.buffer) < header_idx + self.frame_len:
                self.buffer = self.buffer[header_idx:]
                return results

            frame = bytes(self.buffer[header_idx:header_idx + self.frame_len])
            if frame[-1] != TAIL:
                self.buffer = self.buffer[header_idx + 1:]
                continue

            self.buffer = self.buffer[header_idx + self.frame_len:]
            try:
                results.append((frame, self.decoder(frame)))
            except ValueError as exc:
                print("Drop invalid frame {}: {}".format(hex_bytes(frame), exc))


def open_serial(port, baudrate, timeout):
    return serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


def read_serial_chunk(ser, size=64):
    """
    Read serial data without depending on ser.in_waiting.

    Some Orange Pi UART device nodes can be opened by pyserial, but do not
    support the TIOCINQ ioctl used by in_waiting. A timeout-based read is slower
    but works better for this simple link test.
    """
    return ser.read(size)


def run_opi_mode(ser, total_waypoints, interval, start_delay):
    """
    Orange Pi side integration test:
    - receive forbidden-zone frames from the serial assistant
    - wait start_delay seconds after receiving forbidden zones
    - periodically send simulated progress/result frames back
    """
    parser = FrameParser(FORBIDDEN_FRAME_LEN, decode_forbidden_frame)
    waypoint_idx = 0
    last_tx_time = 0.0
    tx_start_time = None

    print("OPI mode on {} @ {} baud".format(ser.port, ser.baudrate))
    print("RX forbidden example: AA A1 B1 A2 B2 A3 B3 FF")
    print("TX status format:    AA idx cls cnt FF")
    print("TX starts {} seconds after the first valid forbidden-zone frame.".format(start_delay))
    print("Press Ctrl+C to stop.")

    while True:
        data = read_serial_chunk(ser)
        if data:
            print("RX raw:", hex_bytes(data))
            parser.feed(data)
            for frame, zones in parser.parse_all():
                print("RX forbidden:", zones, "raw={}".format(hex_bytes(frame)))
                if tx_start_time is None:
                    tx_start_time = time.time() + start_delay
                    print("Forbidden zones received. TX will start after {:.1f}s.".format(start_delay))

        now = time.time()
        if tx_start_time is not None and now >= tx_start_time and now - last_tx_time >= interval:
            if waypoint_idx % 5 == 4:
                frame = build_status_frame(waypoint_idx, animal_cls=0x00, animal_cnt=1)
            else:
                frame = build_status_frame(waypoint_idx, animal_cls=0xFF, animal_cnt=0)
            ser.write(frame)
            ser.flush()
            print("TX status: idx={} raw={}".format(waypoint_idx, hex_bytes(frame)))
            waypoint_idx = (waypoint_idx + 1) % max(total_waypoints, 1)
            last_tx_time = now

        time.sleep(0.02)


def run_rx_mode(ser, kind):
    if kind == "forbidden":
        parser = FrameParser(FORBIDDEN_FRAME_LEN, decode_forbidden_frame)
    else:
        parser = FrameParser(STATUS_FRAME_LEN, decode_status_frame)

    print("RX {} mode on {} @ {} baud".format(kind, ser.port, ser.baudrate))
    while True:
        data = read_serial_chunk(ser)
        if data:
            print("RX raw:", hex_bytes(data))
            parser.feed(data)
            for frame, value in parser.parse_all():
                if kind == "status":
                    idx, animal_cls, cnt = value
                    print(
                        "RX status: idx={} cls=0x{:02X}({}) cnt={} raw={}".format(
                            idx,
                            animal_cls,
                            ANIMAL_NAMES.get(animal_cls, "unknown"),
                            cnt,
                            hex_bytes(frame),
                        )
                    )
                else:
                    print("RX forbidden:", value, "raw={}".format(hex_bytes(frame)))
        time.sleep(0.02)


def run_tx_progress_mode(ser, total_waypoints, interval):
    idx = 0
    print("TX progress mode on {} @ {} baud".format(ser.port, ser.baudrate))
    while True:
        frame = build_status_frame(idx, 0xFF, 0)
        ser.write(frame)
        ser.flush()
        print("TX status: idx={} raw={}".format(idx, hex_bytes(frame)))
        idx = (idx + 1) % max(total_waypoints, 1)
        time.sleep(interval)


def run_tx_forbidden_mode(ser, zones):
    frame = build_forbidden_frame(zones)
    ser.write(frame)
    ser.flush()
    print("TX forbidden:", zones, "raw={}".format(hex_bytes(frame)))


def parse_zone_arg(text):
    parts = text.split(",")
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("zone must look like A1,B1")
    return parts[0].strip(), parts[1].strip()


def main():
    parser = argparse.ArgumentParser(description="DL-20 air-link serial protocol tester")
    parser.add_argument("--port", default="/dev/my_blue", help="serial port, e.g. /dev/ttyS7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=9600, help="serial baud rate")
    parser.add_argument("--timeout", type=float, default=0.05, help="serial read timeout")
    parser.add_argument(
        "--mode",
        choices=["opi", "rx-status", "rx-forbidden", "tx-progress", "tx-forbidden"],
        default="opi",
        help="test mode",
    )
    parser.add_argument("--total", type=int, default=20, help="simulated total waypoint count")
    parser.add_argument("--interval", type=float, default=1.0, help="TX interval in seconds")
    parser.add_argument("--start-delay", type=float, default=5.0, help="seconds to wait before TX after RX forbidden frame")
    parser.add_argument(
        "--zone",
        action="append",
        type=parse_zone_arg,
        default=[],
        help="for tx-forbidden, repeat 3 times, e.g. --zone A1,B1 --zone A2,B2 --zone A3,B3",
    )
    args, unknown_args = parser.parse_known_args()
    if unknown_args:
        print("Ignored extra arguments:", " ".join(unknown_args))

    ser = open_serial(args.port, args.baud, args.timeout)
    try:
        if args.mode == "opi":
            run_opi_mode(ser, args.total, args.interval, args.start_delay)
        elif args.mode == "rx-status":
            run_rx_mode(ser, "status")
        elif args.mode == "rx-forbidden":
            run_rx_mode(ser, "forbidden")
        elif args.mode == "tx-progress":
            run_tx_progress_mode(ser, args.total, args.interval)
        elif args.mode == "tx-forbidden":
            zones = args.zone or [("A1", "B1"), ("A2", "B2"), ("A3", "B3")]
            run_tx_forbidden_mode(ser, zones)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
