# -*- coding: utf-8 -*-
"""
Simple Module
Function: Path simplification utilities
"""


def simplify_path_from_file(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    waypoints = []
    for line in lines:
        line = line.strip()
        if ': ' in line and '(' in line:
            coord = line.split(': ')[1].strip()
            waypoints.append(coord)

    simplified = []
    for wp in waypoints:
        if not simplified or wp != simplified[-1]:
            simplified.append(wp)

    with open(output_file, 'w') as f:
        for wp in simplified:
            f.write(f"({wp})\n")

    print(f"Simplified path written to {output_file}")
    print(f"Original waypoints: {len(waypoints)}, Simplified: {len(simplified)}")
    return simplified