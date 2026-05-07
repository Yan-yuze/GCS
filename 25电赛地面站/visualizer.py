# -*- coding: utf-8 -*-
"""
Path Visualization Module
Function: Visualize the flight path avoiding forbidden zones
"""

import tkinter as tk
from tkinter import ttk
import sys
sys.path.append('.')
from path_generator import generate_flight_plan, parse_coordinate, generate_zigzag_order, START, END

COLS = 9
ROWS = 7
CELL_SIZE = 60
PADDING = 40

GRID_COLOR = '#CCCCCC'
FORBIDDEN_COLOR = '#FF4444'
PATH_COLOR = '#4444FF'
START_END_COLOR = '#44FF44'
VISITED_COLOR = '#8888FF'


class PathVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Drone Path Planner - Visualization")
        self.root.geometry(f"{COLS * CELL_SIZE + PADDING * 2 + 200}x{ROWS * CELL_SIZE + PADDING * 2 + 100}")

        self.canvas = tk.Canvas(root, width=COLS * CELL_SIZE + PADDING * 2,
                                 height=ROWS * CELL_SIZE + PADDING * 2, bg='white')
        self.canvas.pack(side=tk.LEFT, padx=10, pady=10)

        self.control_frame = ttk.Frame(root)
        self.control_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)

        ttk.Label(self.control_frame, text="Forbidden Zones Input", font=('Arial', 12, 'bold')).pack(pady=5)

        ttk.Label(self.control_frame, text="Zone 1 (e.g., A4,B4):").pack(pady=2)
        self.zone1_entry = ttk.Entry(self.control_frame, width=15)
        self.zone1_entry.pack(pady=2)

        ttk.Label(self.control_frame, text="Zone 2 (e.g., A4,B3):").pack(pady=2)
        self.zone2_entry = ttk.Entry(self.control_frame, width=15)
        self.zone2_entry.pack(pady=2)

        ttk.Label(self.control_frame, text="Zone 3 (e.g., A5,B3):").pack(pady=2)
        self.zone3_entry = ttk.Entry(self.control_frame, width=15)
        self.zone3_entry.pack(pady=2)

        self.generate_btn = ttk.Button(self.control_frame, text="Generate Path", command=self.generate_path)
        self.generate_btn.pack(pady=10)

        self.show_btn = ttk.Button(self.control_frame, text="Show Animation", command=self.animate_path, state=tk.DISABLED)
        self.show_btn.pack(pady=5)

        self.clear_btn = ttk.Button(self.control_frame, text="Clear", command=self.clear_all)
        self.clear_btn.pack(pady=5)

        self.info_label = ttk.Label(self.control_frame, text="", wraplength=150)
        self.info_label.pack(pady=10)

        self.forbidden_set = set()
        self.path = []
        self.grid_items = {}

        self.draw_grid()

    def draw_grid(self):
        for row in range(ROWS):
            for col in range(COLS):
                x1 = PADDING + col * CELL_SIZE
                y1 = PADDING + (ROWS - 1 - row) * CELL_SIZE
                x2 = x1 + CELL_SIZE
                y2 = y1 + CELL_SIZE

                rect = self.canvas.create_rectangle(x1, y1, x2, y2, outline=GRID_COLOR, width=1)
                self.grid_items[(col, row)] = {'rect': rect, 'oval': None, 'line': None}

                col_label = chr(ord('A') + col)
                row_label = str(row + 1)
                self.canvas.create_text(x1 + CELL_SIZE/2, y1 + CELL_SIZE/2,
                                        text=f"{col_label}{row_label}", font=('Arial', 8))

        self.canvas.create_text(PADDING/2, PADDING + ROWS * CELL_SIZE/2,
                                 text="Y\\X", font=('Arial', 10))

    def clear_all(self):
        for key, item in self.grid_items.items():
            if item['oval']:
                self.canvas.delete(item['oval'])
                item['oval'] = None
            if item['line']:
                self.canvas.delete(item['line'])
                item['line'] = None
            self.canvas.itemconfig(item['rect'], fill='white')

        self.forbidden_set.clear()
        self.path.clear()
        self.info_label.config(text="")
        self.show_btn.config(state=tk.DISABLED)

    def generate_path(self):
        self.clear_all()

        zone1 = self.zone1_entry.get().strip()
        zone2 = self.zone2_entry.get().strip()
        zone3 = self.zone3_entry.get().strip()

        if not zone1 or not zone2 or not zone3:
            self.info_label.config(text="Please enter all 3 forbidden zones!")
            return

        try:
            z1 = zone1.replace(' ', '').split(',')
            z2 = zone2.replace(' ', '').split(',')
            z3 = zone3.replace(' ', '').split(',')

            if len(z1) != 2 or len(z2) != 2 or len(z3) != 2:
                raise ValueError()

            forbidden_zones = [(z1[0], z1[1]), (z2[0], z2[1]), (z3[0], z3[1])]
        except:
            self.info_label.config(text="Invalid format! Use like: A4,B4")
            return

        for coord in forbidden_zones:
            parsed = parse_coordinate(coord)
            if parsed:
                self.forbidden_set.add(parsed)

        for f in self.forbidden_set:
            col, row = f
            x1 = PADDING + col * CELL_SIZE
            y1 = PADDING + (ROWS - 1 - row) * CELL_SIZE
            x2 = x1 + CELL_SIZE
            y2 = y1 + CELL_SIZE
            self.canvas.itemconfig(self.grid_items[f]['rect'], fill=FORBIDDEN_COLOR)

        generate_flight_plan("A9,B1", forbidden_zones)

        self.load_path_from_file()
        self.draw_static_path()
        self.show_btn.config(state=tk.NORMAL)

        total_cells = COLS * ROWS - len(self.forbidden_set)
        self.info_label.config(text=f"Forbidden: {len(self.forbidden_set)} cells\n"
                                     f"Path length: {len(self.path)} steps\n"
                                     f"Coverage: {len(self.path) - 1}/{total_cells} cells")

    def load_path_from_file(self):
        self.path.clear()
        try:
            with open("flight_path.txt", 'r') as f:
                for line in f:
                    if ': ' in line:
                        coord_str = line.split(': ')[1].strip()
                        if coord_str.startswith('(') and coord_str.endswith(')'):
                            coord_str = coord_str[1:-1]
                            parts = coord_str.split(',')
                            if len(parts) == 2:
                                col = int(parts[0].replace('A', '')) - 1
                                row = int(parts[1].replace('B', '')) - 1
                                self.path.append((col, row))
        except FileNotFoundError:
            pass

    def draw_static_path(self):
        for i in range(len(self.path) - 1):
            p1 = self.path[i]
            p2 = self.path[i + 1]

            x1 = PADDING + p1[0] * CELL_SIZE + CELL_SIZE / 2
            y1 = PADDING + (ROWS - 1 - p1[1]) * CELL_SIZE + CELL_SIZE / 2
            x2 = PADDING + p2[0] * CELL_SIZE + CELL_SIZE / 2
            y2 = PADDING + (ROWS - 1 - p2[1]) * CELL_SIZE + CELL_SIZE / 2

            if p1 == START or p1 == END:
                color = START_END_COLOR
            else:
                color = PATH_COLOR

            line = self.canvas.create_line(x1, y1, x2, y2, fill=color, width=2)
            self.grid_items[p1]['line'] = line

        for i, p in enumerate(self.path):
            if self.grid_items[p]['oval'] is None:
                x = PADDING + p[0] * CELL_SIZE + CELL_SIZE / 2
                y = PADDING + (ROWS - 1 - p[1]) * CELL_SIZE + CELL_SIZE / 2

                if p == START or p == END:
                    color = START_END_COLOR
                    size = 12
                else:
                    color = PATH_COLOR
                    size = 6

                oval = self.canvas.create_oval(x - size, y - size, x + size, y + size, fill=color)
                self.grid_items[p]['oval'] = oval

    def animate_path(self):
        self.clear_all()
        for f in self.forbidden_set:
            self.canvas.itemconfig(self.grid_items[f]['rect'], fill=FORBIDDEN_COLOR)

        self.canvas.update()

        for i in range(len(self.path)):
            p = self.path[i]

            if p not in self.forbidden_set and p != START and p != END:
                x1 = PADDING + p[0] * CELL_SIZE
                y1 = PADDING + (ROWS - 1 - p[1]) * CELL_SIZE
                self.canvas.itemconfig(self.grid_items[p]['rect'], fill=VISITED_COLOR)

            if i > 0:
                p_prev = self.path[i - 1]
                x1 = PADDING + p_prev[0] * CELL_SIZE + CELL_SIZE / 2
                y1 = PADDING + (ROWS - 1 - p_prev[1]) * CELL_SIZE + CELL_SIZE / 2
                x2 = PADDING + p[0] * CELL_SIZE + CELL_SIZE / 2
                y2 = PADDING + (ROWS - 1 - p[1]) * CELL_SIZE + CELL_SIZE / 2

                line = self.canvas.create_line(x1, y1, x2, y2, fill=PATH_COLOR, width=3)
                self.canvas.tag_raise(line)

            x = PADDING + p[0] * CELL_SIZE + CELL_SIZE / 2
            y = PADDING + (ROWS - 1 - p[1]) * CELL_SIZE + CELL_SIZE / 2

            if p == START or p == END:
                size = 15
                color = START_END_COLOR
            else:
                size = 8
                color = PATH_COLOR

            oval = self.canvas.create_oval(x - size, y - size, x + size, y + size, fill=color)
            self.canvas.tag_raise(oval)
            self.canvas.update()

            if p != START and p != END:
                self.canvas.after(50)

        self.canvas.after(500)


def main():
    root = tk.Tk()
    app = PathVisualizer(root)
    root.mainloop()


if __name__ == "__main__":
    main()