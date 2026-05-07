# -*- coding: utf-8 -*-
"""
Test Path Planning Algorithm with Debug
"""

from path_generator import generate_flight_plan, parse_coordinate, label_to_coord, generate_zigzag_order

test_forbidden = [("A4", "B4"), ("A4", "B3"), ("A5", "B3")]

print("Testing path planning with forbidden zones:")
print(f"Forbidden zones: {test_forbidden}")

print("\nParsing forbidden coordinates:")
forbidden_set = set()
for coord in test_forbidden:
    parsed = parse_coordinate(coord)
    print(f"  {coord} -> {parsed}")
    if parsed:
        forbidden_set.add(parsed)

print(f"\nForbidden set: {forbidden_set}")

print("\nGenerating zigzag order:")
zigzag_order = generate_zigzag_order(forbidden_set)
print(f"Zigzag order length: {len(zigzag_order)}")
print(f"Zigzag order: {zigzag_order}")

print("\nChecking if forbidden points are in zigzag order:")
for col, row in forbidden_set:
    if (col, row) in zigzag_order:
        print(f"  ERROR: ({col}, {row}) is in zigzag order!")
    else:
        print(f"  OK: ({col}, {row}) is NOT in zigzag order")

print("\nGenerating flight plan...")

generate_flight_plan("A9,B1", test_forbidden)

print("\nPath generation completed!")