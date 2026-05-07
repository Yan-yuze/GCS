# -*- coding: utf-8 -*-
"""
Path Generator Module
Function: Generate optimal flight path avoiding forbidden zones
"""

import heapq
from collections import deque

COLS = 9
ROWS = 7
START = (8, 0)
END = (8, 0)

DIRECTIONS = [(-1, 0), (1, 0), (0, -1), (0, 1)]


def parse_coordinate(coord_str):
    if isinstance(coord_str, tuple):
        col = int(coord_str[0].replace('A', '')) - 1
        row = int(coord_str[1].replace('B', '')) - 1
        return (col, row)
    coord_str = coord_str.replace('A', '').replace('B', ',')
    parts = coord_str.split(',')
    if len(parts) == 2:
        col = int(parts[0]) - 1
        row = int(parts[1]) - 1
        return (col, row)
    return None


def coord_to_label(col, row):
    return f"(A{col + 1}, B{row + 1})"


def label_to_coord(label):
    label = label.strip()
    if label.startswith('('):
        label = label[1:-1]
    parts = label.split(',')
    if len(parts) == 2:
        col = int(parts[0].replace('A', '')) - 1
        row = int(parts[1].replace('B', '')) - 1
        return (col, row)
    return None


def is_valid(col, row, forbidden_set):
    return 0 <= col < COLS and 0 <= row < ROWS and (col, row) not in forbidden_set


def astar_path(start, end, forbidden_set):
    if start == end:
        return [start]

    open_set = [(0, start)]
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for dx, dy in DIRECTIONS:
            neighbor = (current[0] + dx, current[1] + dy)
            if not is_valid(neighbor[0], neighbor[1], forbidden_set):
                continue

            tentative_g = g_score[current] + 1
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score = tentative_g + abs(neighbor[0] - end[0]) + abs(neighbor[1] - end[1])
                heapq.heappush(open_set, (f_score, neighbor))

    return None


def bfs_path(start, end, forbidden_set):
    if start == end:
        return [start]
    queue = deque()
    queue.append((start, [start]))
    visited = set([start])

    while queue:
        (col, row), path = queue.popleft()
        for dx, dy in DIRECTIONS:
            nr, nc = col + dx, row + dy
            if is_valid(nr, nc, forbidden_set) and (nr, nc) not in visited:
                new_path = path + [(nr, nc)]
                if (nr, nc) == end:
                    return new_path
                queue.append(((nr, nc), new_path))
                visited.add((nr, nc))
    return []


def generate_zigzag_order(forbidden_set):
    order = []
    direction_up = False
    for col in reversed(range(COLS)):
        row_order = range(ROWS - 1, -1, -1) if direction_up else range(ROWS)
        points = [(col, r) for r in row_order if is_valid(col, r, forbidden_set)]
        order.extend(points)
        direction_up = not direction_up
    return order


def generate_flight_plan(start_point_hr, forbidden_zones_tuples):
    forbidden_set = set()
    for coord in forbidden_zones_tuples:
        parsed = parse_coordinate(coord)
        if parsed:
            forbidden_set.add(parsed)

    path = []
    ordered_points = generate_zigzag_order(forbidden_set)

    if START not in ordered_points:
        ordered_points = [START] + ordered_points

    current = START
    if current not in forbidden_set:
        path.append(current)

    for next_point in ordered_points:
        if next_point in forbidden_set or next_point == current:
            continue
        segment = bfs_path(current, next_point, forbidden_set)
        if not segment:
            print(f"Warning: Cannot reach from {coord_to_label(*current)} to {coord_to_label(*next_point)}")
            continue
        path.extend(segment[1:])
        current = next_point

    if current != END:
        return_path = bfs_path(current, END, forbidden_set)
        if return_path:
            path.extend(return_path[1:])
        else:
            print("Warning: Cannot return to start!")

    write_path_to_file(path, "flight_path.txt")


def write_path_to_file(path, filename):
    with open(filename, 'w') as f:
        f.write(f"Start: {coord_to_label(*path[0])}\n")
        for i, (col, row) in enumerate(path):
            f.write(f"Step {i}: {coord_to_label(col, row)}\n")
    print(f"Path written to {filename}")
    print(f"Total steps: {len(path)}")