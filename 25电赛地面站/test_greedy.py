# -*- coding: utf-8 -*-
"""
Test Greedy Path Planning
"""
import sys
sys.path.append('g:/电赛/26电赛准备/25电赛地面站')
from path_generator import generate_flight_plan, parse_coordinate

test_forbidden = [("A4", "B4"), ("A4", "B3"), ("A5", "B3")]

print("Testing greedy path planning:")
print(f"Forbidden zones: {test_forbidden}")
print(f"Total cells: 9x7 = 63")
print(f"Forbidden cells: 3")
print(f"Should visit: 60 cells + return to start")
print()

generate_flight_plan("A9,B1", test_forbidden)

with open("g:/电赛/26电赛准备/25电赛地面站/flight_path.txt", 'r') as f:
    lines = f.readlines()
    print(f"Total path steps: {len(lines) - 1}")  # -1 for "Start:" line