#!/usr/bin/env python3
"""
Initial Pose Picker
===================
Open a ROS map, click-drag to place an arrow indicating the robot's initial
pose, then copy the ready-to-run ros2 topic pub command.

Usage:
    python initial_pose_picker.py                   # auto-detect map in cwd
    python initial_pose_picker.py map.yaml
    python initial_pose_picker.py map.pgm map.yaml

Controls:
    Left drag       — draw pose arrow (start = position, direction = heading)
    Scroll wheel    — zoom in / out
    Middle drag     — pan
    Click "Copy"    — copy command to clipboard
"""

import math
import os
import sys

import yaml

try:
    import tkinter as tk
    from tkinter import messagebox
except ImportError:
    print("tkinter not found — install python3-tk")
    sys.exit(1)

try:
    from PIL import Image, ImageTk
except ImportError:
    print("Pillow not found — install with: pip install Pillow")
    sys.exit(1)

_RESAMPLE = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS


# ── Helpers ───────────────────────────────────────────────────────────────────

def pixel_to_world(px, py, origin, resolution, img_height):
    wx = origin[0] + px * resolution
    wy = origin[1] + (img_height - py) * resolution
    return round(wx, 4), round(wy, 4)


def yaw_to_quaternion(yaw):
    """Yaw angle (radians) → (qx, qy, qz, qw) with qx=qy=0."""
    hz = yaw / 2.0
    return 0.0, 0.0, round(math.sin(hz), 6), round(math.cos(hz), 6)


def build_command(wx, wy, yaw):
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    cov = "[0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.0685]"
    return (
        f"ros2 topic pub --once /initialpose "
        f"geometry_msgs/msg/PoseWithCovarianceStamped \""
        f"{{header: {{frame_id: 'map'}}, "
        f"pose: {{pose: {{"
        f"position: {{x: {wx}, y: {wy}, z: 0.0}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        f"}}, covariance: {cov}}}}}\""
    )


# ── App ───────────────────────────────────────────────────────────────────────

class PosePicker:

    def __init__(self, root: tk.Tk, pgm_path: str, yaml_path: str):
        self.root = root
        root.title("Initial Pose Picker")
        root.configure(bg="#0d1117")

        # Load map metadata
        with open(yaml_path) as f:
            y = yaml.safe_load(f)
        self.resolution = float(y.get("resolution", 0.05))
        self.origin     = list(y.get("origin", [0.0, 0.0, 0.0]))

        self.orig_img = Image.open(pgm_path).convert("RGB")
        self.img_w    = self.orig_img.width
        self.img_h    = self.orig_img.height

        # Zoom / pan state
        self.zoom   = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._pan_anchor = None

        # Drag state
        self._drag_start = None   # canvas coords of press
        self._arrow_id   = None

        self._last_cmd = ""

        self._build_ui()
        root.update_idletasks()
        self._fit_map()

    # ── UI ────────────────────────────────────────────────────────────────────

    def _build_ui(self):
        # Canvas
        cc = tk.Frame(self.root, bg="#0d1117")
        cc.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        self.canvas = tk.Canvas(cc, bg="#1a1a2e", cursor="crosshair",
                                highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Status bar
        self._status_var = tk.StringVar(value="Click and drag on the map to set initial pose")
        tk.Label(self.root, textvariable=self._status_var,
                 bg="#0d1117", fg="#58a6ff", font=("Helvetica", 8),
                 anchor=tk.W).pack(fill=tk.X, padx=8, pady=(0, 2))

        # Command display + copy button
        bot = tk.Frame(self.root, bg="#0d1117")
        bot.pack(fill=tk.X, padx=6, pady=(0, 6))

        self._cmd_var = tk.StringVar()
        tk.Entry(bot, textvariable=self._cmd_var,
                 bg="#161b22", fg="#3fb950", insertbackground="white",
                 relief=tk.FLAT, font=("Courier", 8),
                 state="readonly").pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 6))

        tk.Button(bot, text="Copy", command=self._copy,
                  bg="#1f6feb", fg="white", relief=tk.FLAT,
                  padx=10, font=("Helvetica", 8)).pack(side=tk.RIGHT)

        # Bindings
        c = self.canvas
        c.bind("<ButtonPress-1>",   self._on_press)
        c.bind("<B1-Motion>",       self._on_drag)
        c.bind("<ButtonRelease-1>", self._on_release)
        c.bind("<ButtonPress-2>",   self._on_pan_press)
        c.bind("<B2-Motion>",       self._on_pan_drag)
        c.bind("<MouseWheel>",      self._on_scroll)
        c.bind("<Button-4>",        self._on_scroll)
        c.bind("<Button-5>",        self._on_scroll)

    # ── Zoom / pan ────────────────────────────────────────────────────────────

    def _fit_map(self):
        self.root.update()
        cw = self.canvas.winfo_width()  or 900
        ch = self.canvas.winfo_height() or 600
        self.zoom   = min(cw / self.img_w, ch / self.img_h, 4.0)
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._redraw()

    def _on_scroll(self, event):
        factor = 1.12
        if event.num == 5 or (hasattr(event, "delta") and event.delta < 0):
            factor = 1 / factor
        old    = self.zoom
        self.zoom = max(0.05, min(16.0, self.zoom * factor))
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        scale    = self.zoom / old
        self._pan_x = cx - scale * (cx - self._pan_x)
        self._pan_y = cy - scale * (cy - self._pan_y)
        self._redraw()

    def _on_pan_press(self, event):
        self._pan_anchor = (event.x, event.y, self._pan_x, self._pan_y)

    def _on_pan_drag(self, event):
        if self._pan_anchor:
            ax, ay, apx, apy = self._pan_anchor
            self._pan_x = apx + (event.x - ax)
            self._pan_y = apy + (event.y - ay)
            self._redraw()

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _canvas_to_pixel(self, cx, cy):
        return (cx - self._pan_x) / self.zoom, (cy - self._pan_y) / self.zoom

    def _pixel_to_canvas(self, px, py):
        return px * self.zoom + self._pan_x, py * self.zoom + self._pan_y

    def _canvas_to_world(self, cx, cy):
        px, py = self._canvas_to_pixel(cx, cy)
        return pixel_to_world(px, py, self.origin, self.resolution, self.img_h)

    # ── Drawing ───────────────────────────────────────────────────────────────

    def _redraw(self):
        self.canvas.delete("all")
        iw = max(1, int(self.img_w * self.zoom))
        ih = max(1, int(self.img_h * self.zoom))
        scaled = self.orig_img.resize((iw, ih), _RESAMPLE)
        self._tk_img = ImageTk.PhotoImage(scaled)
        self.canvas.create_image(self._pan_x, self._pan_y,
                                  anchor=tk.NW, image=self._tk_img)

    def _draw_arrow(self, sx, sy, ex, ey):
        if self._arrow_id:
            self.canvas.delete(self._arrow_id)
        self._arrow_id = self.canvas.create_line(
            sx, sy, ex, ey,
            fill="#FF4500", width=3, arrow=tk.LAST,
            arrowshape=(16, 20, 7))

    # ── Mouse events ──────────────────────────────────────────────────────────

    def _on_press(self, event):
        self._drag_start = (self.canvas.canvasx(event.x),
                            self.canvas.canvasy(event.y))
        if self._arrow_id:
            self.canvas.delete(self._arrow_id)
            self._arrow_id = None

    def _on_drag(self, event):
        if not self._drag_start:
            return
        ex = self.canvas.canvasx(event.x)
        ey = self.canvas.canvasy(event.y)
        sx, sy = self._drag_start
        self._draw_arrow(sx, sy, ex, ey)

    def _on_release(self, event):
        if not self._drag_start:
            return
        sx, sy = self._drag_start
        ex = self.canvas.canvasx(event.x)
        ey = self.canvas.canvasy(event.y)
        self._drag_start = None

        # Need a meaningful drag
        if math.hypot(ex - sx, ey - sy) < 5:
            return

        wx, wy = self._canvas_to_world(sx, sy)

        # Canvas y increases downward; world y increases upward → flip dy
        dx = (ex - sx) / self.zoom
        dy = (sy - ey) / self.zoom
        yaw = math.atan2(dy, dx)

        cmd = build_command(wx, wy, yaw)
        self._last_cmd = cmd
        self._cmd_var.set(cmd)
        print("\n" + cmd + "\n")

        self._status_var.set(
            f"x={wx} m   y={wy} m   yaw={math.degrees(yaw):.1f}°  "
            f"(z={yaw_to_quaternion(yaw)[2]:.4f}  w={yaw_to_quaternion(yaw)[3]:.4f})"
        )

    def _copy(self):
        if not self._last_cmd:
            return
        self.root.clipboard_clear()
        self.root.clipboard_append(self._last_cmd)
        self._status_var.set("Copied to clipboard!")


# ── Entry point ───────────────────────────────────────────────────────────────

def _find_map_in_cwd():
    for fname in os.listdir("."):
        if fname.endswith((".yaml", ".yml")):
            stem = os.path.splitext(fname)[0]
            pgm  = stem + ".pgm"
            if os.path.exists(pgm):
                return pgm, fname
    return None, None


def main():
    pgm, yml = None, None

    if len(sys.argv) == 3:
        pgm, yml = sys.argv[1], sys.argv[2]
    elif len(sys.argv) == 2:
        arg = sys.argv[1]
        if arg.endswith((".yaml", ".yml")):
            yml = arg
            pgm  = os.path.splitext(arg)[0] + ".pgm"
        elif arg.endswith(".pgm"):
            pgm  = arg
            yml  = os.path.splitext(arg)[0] + ".yaml"
    else:
        pgm, yml = _find_map_in_cwd()

    if not pgm or not yml or not os.path.exists(pgm) or not os.path.exists(yml):
        print(__doc__)
        print("ERROR: No map found. Pass map.yaml or place map pair in current directory.")
        sys.exit(1)

    root = tk.Tk()
    root.geometry("1000x700")
    root.configure(bg="#0d1117")
    PosePicker(root, pgm, yml)
    root.mainloop()


if __name__ == "__main__":
    main()
