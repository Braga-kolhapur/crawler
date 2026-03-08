#!/usr/bin/env python3
"""
Coverage Path Planner
=====================
Interactive tool for drawing cleaning zones on a ROS map and generating
boustrophedon (lawnmower) coverage paths for each zone.

Usage:
    python coverage_planner.py                        # auto-detect map in cwd
    python coverage_planner.py map.yaml               # load specific map
    python coverage_planner.py map.pgm map.yaml       # explicit paths

Requires:
    pip install Pillow pyyaml

Mouse controls on the map canvas:
    Left drag        — draw a new zone rectangle
    Left click zone  — select it (shows properties in right panel)
    Right-click zone — context menu (generate path / delete)
    Scroll wheel     — zoom in / out (centered on cursor)
    Middle drag      — pan the map
    Delete key       — delete selected zone
    Ctrl+Z           — undo last zone
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import yaml
import json
import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional

try:
    from PIL import Image, ImageTk
except ImportError:
    print("Pillow not found.  Install with:  pip install Pillow")
    sys.exit(1)

# ──────────────────────────────────────────────────────────────────────────────
# Constants
# ──────────────────────────────────────────────────────────────────────────────

ZONE_COLORS = [
    "#E74C3C", "#3498DB", "#2ECC71", "#F39C12", "#9B59B6",
    "#1ABC9C", "#E67E22", "#E91E63", "#00BCD4", "#8BC34A",
    "#FF5722", "#607D8B", "#795548", "#FFC107",
]

PATH_TYPES = ["boustrophedon"]          # extend here for spiral, etc.
MIN_RECT_PX = 8                         # minimum drawn rectangle (canvas px)
DEFAULT_ROBOT_WIDTH = 0.30              # metres

# Pillow resampling — handle API change between Pillow 9 and 10
_RESAMPLE = Image.Resampling.LANCZOS if hasattr(Image, "Resampling") else Image.LANCZOS


# ──────────────────────────────────────────────────────────────────────────────
# Data classes
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class MapInfo:
    image_path: str
    yaml_path: str
    resolution: float           # metres per pixel
    origin: List[float]         # [x, y, theta]  — bottom-left of map in world
    width: int                  # pixels
    height: int                 # pixels
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196

    def pixel_to_world(self, px: float, py: float) -> Tuple[float, float]:
        """Pixel (col, row) → world (x, y) metres.  Row 0 is image top."""
        wx = self.origin[0] + px * self.resolution
        wy = self.origin[1] + (self.height - py) * self.resolution
        return round(wx, 4), round(wy, 4)

    def world_to_pixel(self, wx: float, wy: float) -> Tuple[float, float]:
        """World (x, y) metres → pixel (col, row)."""
        px = (wx - self.origin[0]) / self.resolution
        py = self.height - (wy - self.origin[1]) / self.resolution
        return px, py


@dataclass
class CoverageZone:
    key: str
    x1: int                             # pixel coords (as drawn — may be unordered)
    y1: int
    x2: int
    y2: int
    path_type: str = "boustrophedon"
    robot_width: float = DEFAULT_ROBOT_WIDTH   # metres
    angle: float = 0.0                  # stripe direction, degrees
    rect_angle: float = 0.0             # rotation of the drawn rectangle itself, degrees
    color: str = "#E74C3C"
    waypoints: List[Dict] = field(default_factory=list)

    # Normalised bounds (always left ≤ right, top ≤ bottom in pixel space)
    @property
    def left(self):   return min(self.x1, self.x2)
    @property
    def top(self):    return min(self.y1, self.y2)
    @property
    def right(self):  return max(self.x1, self.x2)
    @property
    def bottom(self): return max(self.y1, self.y2)

    def corners_px(self) -> List[Tuple[float, float]]:
        """4 corners of the (possibly rotated) rectangle in pixel space: TL, TR, BR, BL."""
        cx = (self.x1 + self.x2) / 2.0
        cy = (self.y1 + self.y2) / 2.0
        hw = abs(self.x2 - self.x1) / 2.0
        hh = abs(self.y2 - self.y1) / 2.0
        a  = math.radians(self.rect_angle)
        ca, sa = math.cos(a), math.sin(a)
        offsets = [(-hw, -hh), (hw, -hh), (hw, hh), (-hw, hh)]
        return [(cx + dx * ca - dy * sa, cy + dx * sa + dy * ca) for dx, dy in offsets]

    def contains_point(self, px: float, py: float) -> bool:
        cx = (self.x1 + self.x2) / 2.0
        cy = (self.y1 + self.y2) / 2.0
        dx, dy = px - cx, py - cy
        a  = math.radians(-self.rect_angle)
        ca, sa = math.cos(a), math.sin(a)
        lx = dx * ca - dy * sa
        ly = dx * sa + dy * ca
        return (abs(lx) <= abs(self.x2 - self.x1) / 2.0 and
                abs(ly) <= abs(self.y2 - self.y1) / 2.0)

    def to_dict(self, map_info: MapInfo) -> dict:
        wl, wb = map_info.pixel_to_world(self.left,  self.bottom)
        wr, wt = map_info.pixel_to_world(self.right, self.top)
        w_m = abs(self.right - self.left) * map_info.resolution
        h_m = abs(self.bottom - self.top) * map_info.resolution
        return {
            "key":           self.key,
            "pixel_bounds":  {"x1": self.left, "y1": self.top,
                              "x2": self.right, "y2": self.bottom},
            "world_bounds":  {"x_min": wl, "y_min": wb,
                              "x_max": wr, "y_max": wt},
            "dimensions_m":  {"width": round(w_m, 3), "height": round(h_m, 3)},
            "path_type":     self.path_type,
            "robot_width_m": self.robot_width,
            "angle_deg":     self.angle,
            "rect_angle_deg": self.rect_angle,
            "color":         self.color,
            "waypoints":     self.waypoints,
        }


# ──────────────────────────────────────────────────────────────────────────────
# Path generation
# ──────────────────────────────────────────────────────────────────────────────

class PathGenerator:
    """Generates coverage waypoints for a CoverageZone."""

    @staticmethod
    def generate(zone: CoverageZone, map_info: MapInfo) -> List[Dict]:
        if zone.path_type == "boustrophedon":
            return PathGenerator.boustrophedon(zone, map_info)
        return []

    @staticmethod
    def _clip_stripe_to_polygon(local_corners: List[Tuple], sy: float):
        """
        Return (x_left, x_right) where the horizontal line y=sy intersects
        the convex polygon defined by local_corners, or None if no intersection.

        This is the key fix for angled stripes: instead of using the bounding-box
        extents (which are wider than the actual rotated rectangle), we find the
        exact x range at each stripe y so waypoints never leave the drawn zone.
        """
        xs = []
        n = len(local_corners)
        for i in range(n):
            x1, y1 = local_corners[i]
            x2, y2 = local_corners[(i + 1) % n]
            if abs(y2 - y1) < 1e-10:
                continue                          # skip horizontal edges
            if min(y1, y2) - 1e-9 <= sy <= max(y1, y2) + 1e-9:
                t  = (sy - y1) / (y2 - y1)
                xs.append(x1 + t * (x2 - x1))
        if len(xs) < 2:
            return None
        return min(xs), max(xs)

    @staticmethod
    def boustrophedon(zone: CoverageZone, map_info: MapInfo) -> List[Dict]:
        """
        Lawnmower / boustrophedon coverage.

        Strategy
        --------
        1. Convert the axis-aligned rectangle to world coordinates.
        2. Rotate the 4 world corners into a local frame defined by `angle`
           so that stripes run horizontally in that frame.
        3. For each stripe y, clip the stripe against the actual rotated polygon
           (not the bounding box) so waypoints stay inside the drawn zone.
        4. Alternate left-right for adjacent stripes.
        5. Rotate waypoints back to the world frame.

        Waypoint dict fields:
            x, y    — world metres
            yaw     — heading in radians (for ROS Pose orientation)
            stripe  — stripe index (0-based)
            type    — "coverage" | "transition"
        """
        # World corners (account for rect_angle rotation): TL, TR, BR, BL
        corners = [map_info.pixel_to_world(px, py) for px, py in zone.corners_px()]

        a = math.radians(zone.angle)
        ca, sa = math.cos(a), math.sin(a)

        def to_local(x, y):
            # rotate by -a
            return x * ca + y * sa, -x * sa + y * ca

        def to_world(lx, ly):
            # rotate by +a
            return lx * ca - ly * sa, lx * sa + ly * ca

        lc = [to_local(x, y) for x, y in corners]
        ly_min = min(p[1] for p in lc)
        ly_max = max(p[1] for p in lc)

        robot_w = max(zone.robot_width, 0.01)

        # Build stripe Y positions (centre of each strip)
        stripe_ys: List[float] = []
        y = ly_min + robot_w / 2.0
        while y <= ly_max + 1e-9:
            stripe_ys.append(y)
            y += robot_w
        if not stripe_ys:
            stripe_ys = [(ly_min + ly_max) / 2.0]

        waypoints: List[Dict] = []
        prev_wx, prev_wy = None, None

        for i, sy in enumerate(stripe_ys):
            # Clip the stripe to the actual rotated rectangle (not bounding box).
            # This prevents waypoints from escaping the drawn zone at any angle.
            x_range = PathGenerator._clip_stripe_to_polygon(lc, sy)
            if x_range is None:
                continue
            lx_lo, lx_hi = x_range

            lx_start, lx_end = (lx_lo, lx_hi) if i % 2 == 0 else (lx_hi, lx_lo)

            wx1, wy1 = to_world(lx_start, sy)
            wx2, wy2 = to_world(lx_end,   sy)

            heading = math.atan2(wy2 - wy1, wx2 - wx1)

            # Optional transition waypoint from end of previous stripe
            if prev_wx is not None and (prev_wx != wx1 or prev_wy != wy1):
                trans_heading = math.atan2(wy1 - prev_wy, wx1 - prev_wx)
                waypoints.append({
                    "x": round(wx1, 4), "y": round(wy1, 4),
                    "yaw": round(trans_heading, 4),
                    "stripe": i, "type": "transition",
                })

            waypoints.append({
                "x": round(wx1, 4), "y": round(wy1, 4),
                "yaw": round(heading, 4),
                "stripe": i, "type": "coverage",
            })
            waypoints.append({
                "x": round(wx2, 4), "y": round(wy2, 4),
                "yaw": round(heading, 4),
                "stripe": i, "type": "coverage",
            })
            prev_wx, prev_wy = wx2, wy2

        return waypoints

    @staticmethod
    def path_length_m(waypoints: List[Dict]) -> float:
        total = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i]["x"] - waypoints[i - 1]["x"]
            dy = waypoints[i]["y"] - waypoints[i - 1]["y"]
            total += math.hypot(dx, dy)
        return total


# ──────────────────────────────────────────────────────────────────────────────
# Main application
# ──────────────────────────────────────────────────────────────────────────────

class CoveragePlannerApp:

    def __init__(self, root: tk.Tk, map_pgm: str = None, map_yaml: str = None):
        self.root = root
        self.root.title("Coverage Path Planner")
        self.root.geometry("1280x820")
        self.root.configure(bg="#0d1117")

        # State
        self.map_info: Optional[MapInfo] = None
        self.original_image: Optional[Image.Image] = None
        self._tk_image: Optional[ImageTk.PhotoImage] = None   # keep reference!

        self.zones: Dict[str, CoverageZone] = {}
        self.sequence: List[str] = []
        self.selected_key: Optional[str] = None

        # Tool mode: "draw" (zone rectangle) | "erase" (clear pixels)
        self._tool_mode: str = "draw"

        # Drawing state
        self._drawing = False
        self._draw_start_px: Optional[Tuple[float, float]] = None
        self._rubber_band_id: Optional[int] = None

        # Image undo stack — stores PIL Image copies before each erase operation
        self._img_undo_stack: List[Image.Image] = []

        # Zoom / pan
        self.zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._pan_anchor: Optional[Tuple[float, float, float, float]] = None

        # Shared vars
        self.show_paths_var   = tk.BooleanVar(value=True)
        self.show_numbers_var = tk.BooleanVar(value=True)
        self.path_type_var    = tk.StringVar(value="boustrophedon")
        self.robot_width_var  = tk.StringVar(value=f"{DEFAULT_ROBOT_WIDTH:.2f}")
        self._angle_dvar      = tk.DoubleVar(value=0.0)
        self.angle_str_var    = tk.StringVar(value="0.0")
        self._angle_dvar.trace_add("write", self._sync_angle_entry_from_scale)
        self._rect_angle_var  = tk.DoubleVar(value=0.0)
        self.rect_angle_str_var = tk.StringVar(value="0.0")

        self._build_ui()
        self._bind_events()

        if map_pgm and map_yaml and os.path.exists(map_pgm) and os.path.exists(map_yaml):
            self._load_map(map_pgm, map_yaml)
        else:
            self._set_status("Ready — open a map.yaml to begin.")

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Menu bar ──────────────────────────────────────────────────────────
        menubar = tk.Menu(self.root)

        fm = tk.Menu(menubar, tearoff=0)
        fm.add_command(label="Open Map…",           command=self._open_map_dialog,   accelerator="Ctrl+O")
        fm.add_separator()
        fm.add_command(label="Save Plan (JSON)…",   command=self._save_plan,         accelerator="Ctrl+S")
        fm.add_command(label="Load Plan…",          command=self._load_plan)
        fm.add_separator()
        fm.add_command(label="Export Waypoints CSV…", command=self._export_csv)
        fm.add_separator()
        fm.add_command(label="Quit",                command=self.root.quit)
        menubar.add_cascade(label="File", menu=fm)

        em = tk.Menu(menubar, tearoff=0)
        em.add_command(label="Undo Last Zone",      command=self._undo_last,          accelerator="Ctrl+Z")
        em.add_command(label="Undo Map Edit",       command=self._undo_map_edit,      accelerator="Ctrl+Shift+Z")
        em.add_separator()
        em.add_command(label="Delete Selected",     command=self._delete_selected,    accelerator="Del")
        em.add_command(label="Clear All Zones",     command=self._clear_all)
        em.add_separator()
        em.add_command(label="Save Edited Map…",    command=self._save_map_dialog)
        menubar.add_cascade(label="Edit", menu=em)

        vm = tk.Menu(menubar, tearoff=0)
        vm.add_checkbutton(label="Show Paths",      variable=self.show_paths_var,    command=self._redraw)
        vm.add_checkbutton(label="Show Zone Labels",variable=self.show_numbers_var,  command=self._redraw)
        vm.add_separator()
        vm.add_command(label="Fit Map to Window",   command=self._fit_map)
        vm.add_command(label="Reset Zoom (100%)",   command=self._reset_zoom)
        menubar.add_cascade(label="View", menu=vm)

        pm = tk.Menu(menubar, tearoff=0)
        pm.add_command(label="Generate All Paths",      command=self._generate_all)
        pm.add_command(label="Generate Selected Path",  command=self._generate_selected)
        menubar.add_cascade(label="Paths", menu=pm)

        self.root.config(menu=menubar)

        # ── Top-level paned window ────────────────────────────────────────────
        paned = tk.PanedWindow(self.root, orient=tk.HORIZONTAL,
                               sashwidth=5, bg="#21262d")
        paned.pack(fill=tk.BOTH, expand=True)

        # ── Left: canvas area ─────────────────────────────────────────────────
        left = tk.Frame(paned, bg="#161b22")
        paned.add(left, minsize=640)

        # Toolbar
        tb = tk.Frame(left, bg="#21262d", pady=3)
        tb.pack(side=tk.TOP, fill=tk.X)

        def tb_btn(text, cmd, bg="#0f3460"):
            return tk.Button(tb, text=text, command=cmd,
                             bg=bg, fg="white", relief=tk.FLAT,
                             padx=7, pady=2, font=("Helvetica", 8))

        tb_btn("Open Map",     self._open_map_dialog).pack(side=tk.LEFT, padx=3, pady=2)
        tb_btn("Generate All", self._generate_all,    bg="#533483").pack(side=tk.LEFT, padx=2)
        tb_btn("Save Plan",    self._save_plan,        bg="#1a7f37").pack(side=tk.LEFT, padx=2)

        tk.Frame(tb, bg="#444", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=4)

        # ── Tool mode toggle ──────────────────────────────────────────────────
        tk.Label(tb, text="Tool:", bg="#21262d", fg="#aaa",
                 font=("Helvetica", 8)).pack(side=tk.LEFT, padx=(2, 1))

        # Keep references so we can re-colour them when mode changes
        self._btn_draw  = tk.Button(tb, text="Draw Zone",
                                    command=lambda: self._set_tool_mode("draw"),
                                    bg="#1f6feb", fg="white", relief=tk.FLAT,
                                    padx=7, pady=2, font=("Helvetica", 8, "bold"))
        self._btn_draw.pack(side=tk.LEFT, padx=1, pady=2)

        self._btn_erase = tk.Button(tb, text="Erase Pixels",
                                    command=lambda: self._set_tool_mode("erase"),
                                    bg="#21262d", fg="#aaa", relief=tk.FLAT,
                                    padx=7, pady=2, font=("Helvetica", 8))
        self._btn_erase.pack(side=tk.LEFT, padx=1, pady=2)

        tk.Button(tb, text="Save Map…",
                  command=self._save_map_dialog,
                  bg="#21262d", fg="#aaa", relief=tk.FLAT,
                  padx=5, pady=2, font=("Helvetica", 8)).pack(side=tk.LEFT, padx=2)

        tk.Frame(tb, bg="#444", width=1).pack(side=tk.LEFT, fill=tk.Y, padx=6, pady=4)

        tk.Label(tb, text="Zoom:", bg="#21262d", fg="#aaa",
                 font=("Helvetica", 8)).pack(side=tk.LEFT)
        tk.Button(tb, text="−", command=self._zoom_out,
                  bg="#21262d", fg="white", relief=tk.FLAT,
                  width=2, font=("Helvetica", 10)).pack(side=tk.LEFT, padx=1)
        self._zoom_label = tk.Label(tb, text="100%", bg="#21262d",
                                    fg="white", width=5, font=("Helvetica", 8))
        self._zoom_label.pack(side=tk.LEFT)
        tk.Button(tb, text="+", command=self._zoom_in,
                  bg="#21262d", fg="white", relief=tk.FLAT,
                  width=2, font=("Helvetica", 10)).pack(side=tk.LEFT, padx=1)
        tk.Button(tb, text="Fit",  command=self._fit_map,
                  bg="#21262d", fg="white", relief=tk.FLAT,
                  padx=5, font=("Helvetica", 8)).pack(side=tk.LEFT, padx=2)

        tk.Checkbutton(tb, text="Show paths", variable=self.show_paths_var,
                       command=self._redraw, bg="#21262d", fg="#aaa",
                       selectcolor="#21262d", activebackground="#21262d",
                       font=("Helvetica", 8)).pack(side=tk.RIGHT, padx=8)

        # Canvas + scrollbars
        cc = tk.Frame(left, bg="#0d1117")
        cc.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(cc, bg="#0d1117", cursor="crosshair",
                                highlightthickness=0)
        hbar = ttk.Scrollbar(cc, orient=tk.HORIZONTAL, command=self.canvas.xview)
        vbar = ttk.Scrollbar(cc, orient=tk.VERTICAL,   command=self.canvas.yview)
        self.canvas.configure(xscrollcommand=hbar.set, yscrollcommand=vbar.set)

        hbar.pack(side=tk.BOTTOM, fill=tk.X)
        vbar.pack(side=tk.RIGHT,  fill=tk.Y)
        self.canvas.pack(fill=tk.BOTH, expand=True)

        # Status / cursor bars
        self._status_var = tk.StringVar(value="")
        tk.Label(left, textvariable=self._status_var, anchor=tk.W,
                 bg="#0d1117", fg="#58a6ff", font=("Helvetica", 8),
                 padx=6, pady=2).pack(side=tk.BOTTOM, fill=tk.X)

        self._cursor_var = tk.StringVar(value="")
        tk.Label(left, textvariable=self._cursor_var, anchor=tk.E,
                 bg="#0d1117", fg="#6e7681", font=("Courier", 8),
                 padx=6).pack(side=tk.BOTTOM, fill=tk.X)

        # ── Right: control panel ──────────────────────────────────────────────
        right = tk.Frame(paned, bg="#0d1117", width=310)
        paned.add(right, minsize=280)

        def section(parent, title):
            f = tk.LabelFrame(parent, text=f"  {title}  ",
                              bg="#0d1117", fg="#58a6ff",
                              font=("Helvetica", 8, "bold"),
                              relief=tk.GROOVE)
            f.pack(fill=tk.X, padx=8, pady=(6, 2))
            return f

        # Map info
        mi_sec = section(right, "Map Info")
        self._map_info_text = tk.Text(mi_sec, height=5, state=tk.DISABLED,
                                      bg="#161b22", fg="#c9d1d9",
                                      font=("Courier", 8), relief=tk.FLAT,
                                      padx=4, pady=2)
        self._map_info_text.pack(fill=tk.X, padx=4, pady=4)

        # Zone list
        zl_sec = section(right, "Zones  (drag rows to reorder)")
        zl_frame = tk.Frame(zl_sec, bg="#0d1117")
        zl_frame.pack(fill=tk.X, padx=4, pady=(4, 2))

        self._zone_lb = tk.Listbox(zl_frame, bg="#161b22", fg="#c9d1d9",
                                    selectbackground="#1f6feb", height=7,
                                    font=("Courier", 8), relief=tk.FLAT,
                                    exportselection=False)
        zl_sb = ttk.Scrollbar(zl_frame, command=self._zone_lb.yview)
        self._zone_lb.configure(yscrollcommand=zl_sb.set)
        self._zone_lb.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        zl_sb.pack(side=tk.RIGHT, fill=tk.Y)
        self._zone_lb.bind("<<ListboxSelect>>", self._on_lb_select)

        zl_btns = tk.Frame(zl_sec, bg="#0d1117")
        zl_btns.pack(fill=tk.X, padx=4, pady=(0, 4))
        for txt, cmd in [("↑ Up", self._move_up), ("↓ Down", self._move_down)]:
            tk.Button(zl_btns, text=txt, command=cmd,
                      bg="#21262d", fg="white", relief=tk.FLAT,
                      padx=5, pady=2, font=("Helvetica", 8)).pack(side=tk.LEFT, padx=2)
        tk.Button(zl_btns, text="Delete", command=self._delete_selected,
                  bg="#b91c1c", fg="white", relief=tk.FLAT,
                  padx=5, pady=2, font=("Helvetica", 8)).pack(side=tk.RIGHT, padx=2)

        # Zone properties
        zp_sec = section(right, "Zone Properties")
        zp_sec.columnconfigure(1, weight=1)

        def lbl(text, row):
            tk.Label(zp_sec, text=text, bg="#0d1117", fg="#8b949e",
                     font=("Helvetica", 8), anchor=tk.W).grid(
                row=row, column=0, sticky=tk.W, padx=(8, 4), pady=3)

        def entry_w(var, row, validate_cmd=None):
            e = tk.Entry(zp_sec, textvariable=var, bg="#161b22", fg="white",
                         insertbackground="white", relief=tk.FLAT,
                         font=("Helvetica", 9), width=10)
            e.grid(row=row, column=1, sticky=tk.EW, padx=(0, 8), pady=3)
            return e

        lbl("Path type:",        0)
        path_cb = ttk.Combobox(zp_sec, textvariable=self.path_type_var,
                               values=PATH_TYPES, state="readonly", width=14)
        path_cb.grid(row=0, column=1, sticky=tk.EW, padx=(0, 8), pady=3)
        path_cb.bind("<<ComboboxSelected>>", self._on_props_change)

        lbl("Robot width (m):",  1)
        rw_entry = entry_w(self.robot_width_var, 1)
        rw_entry.bind("<Return>",   self._on_props_change)
        rw_entry.bind("<FocusOut>", self._on_props_change)

        lbl("Stripe angle (°):", 2)
        angle_frame = tk.Frame(zp_sec, bg="#0d1117")
        angle_frame.grid(row=2, column=1, sticky=tk.EW, padx=(0, 8), pady=3)
        angle_entry = tk.Entry(angle_frame, textvariable=self.angle_str_var,
                               bg="#161b22", fg="white", insertbackground="white",
                               relief=tk.FLAT, font=("Helvetica", 9), width=10)
        angle_entry.pack(side=tk.LEFT, fill=tk.X, expand=True)
        angle_entry.bind("<Return>",   self._on_angle_entry_commit)
        angle_entry.bind("<FocusOut>", self._on_angle_entry_commit)

        # Angle slider (0–180°)
        self._angle_scale = tk.Scale(zp_sec, from_=0, to=180, orient=tk.HORIZONTAL,
                                      variable=self._angle_dvar,
                                      bg="#0d1117", fg="#c9d1d9", troughcolor="#21262d",
                                      highlightthickness=0, showvalue=False,
                                      command=self._on_scale_moved)
        self._angle_scale.grid(row=3, column=0, columnspan=2,
                               sticky=tk.EW, padx=8, pady=(0, 4))

        # Rectangle orientation
        lbl("Rect orient (°):", 4)
        rect_frame = tk.Frame(zp_sec, bg="#0d1117")
        rect_frame.grid(row=4, column=1, sticky=tk.EW, padx=(0, 8), pady=3)
        rect_entry = tk.Entry(rect_frame, textvariable=self.rect_angle_str_var,
                              bg="#161b22", fg="white", insertbackground="white",
                              relief=tk.FLAT, font=("Helvetica", 9), width=7)
        rect_entry.pack(side=tk.LEFT)
        rect_entry.bind("<Return>",   self._on_rect_angle_commit)
        rect_entry.bind("<FocusOut>", self._on_rect_angle_commit)

        tk.Button(rect_frame, text="▲",
                  command=lambda: self._rotate_rect(+15),
                  bg="#21262d", fg="white", relief=tk.FLAT,
                  width=2, font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(4, 1))
        tk.Button(rect_frame, text="▼",
                  command=lambda: self._rotate_rect(-15),
                  bg="#21262d", fg="white", relief=tk.FLAT,
                  width=2, font=("Helvetica", 9)).pack(side=tk.LEFT, padx=(1, 0))

        # Path length indicator
        self._path_len_var = tk.StringVar(value="No path generated yet")
        tk.Label(zp_sec, textvariable=self._path_len_var,
                 bg="#0d1117", fg="#3fb950", font=("Helvetica", 7),
                 anchor=tk.W).grid(row=5, column=0, columnspan=2,
                                   sticky=tk.W, padx=8, pady=(0, 4))

        tk.Button(zp_sec, text="Generate Path for Selected Zone",
                  command=self._generate_selected,
                  bg="#1f6feb", fg="white", relief=tk.FLAT,
                  pady=4, font=("Helvetica", 8)).grid(
            row=6, column=0, columnspan=2, sticky=tk.EW, padx=8, pady=(0, 8))

        # Sequence
        seq_sec = section(right, "Execution Sequence")
        sf = tk.Frame(seq_sec, bg="#0d1117")
        sf.pack(fill=tk.X, padx=4, pady=4)
        tk.Label(sf, text="Order:", bg="#0d1117", fg="#8b949e",
                 font=("Helvetica", 8)).pack(side=tk.LEFT, padx=4)
        self._seq_var = tk.StringVar()
        tk.Entry(sf, textvariable=self._seq_var,
                 bg="#161b22", fg="white", insertbackground="white",
                 relief=tk.FLAT, font=("Helvetica", 9)).pack(
            side=tk.LEFT, fill=tk.X, expand=True, padx=4)
        tk.Label(seq_sec, text="Comma-separated keys, e.g.  a,b,c,d",
                 bg="#0d1117", fg="#484f58",
                 font=("Helvetica", 7)).pack(padx=8, pady=(0, 4), anchor=tk.W)

        # Actions
        act_sec = section(right, "Actions")
        ag = tk.Frame(act_sec, bg="#0d1117")
        ag.pack(fill=tk.X, padx=4, pady=4)
        ag.columnconfigure(0, weight=1)
        ag.columnconfigure(1, weight=1)

        def action_btn(text, cmd, bg, row, col, colspan=1):
            tk.Button(ag, text=text, command=cmd,
                      bg=bg, fg="white", relief=tk.FLAT,
                      pady=4, font=("Helvetica", 8)).grid(
                row=row, column=col, columnspan=colspan,
                sticky=tk.EW, padx=2, pady=2)

        action_btn("Generate All Paths", self._generate_all, "#1f6feb", 0, 0, 2)
        action_btn("Save Plan (JSON)",   self._save_plan,    "#1a7f37", 1, 0)
        action_btn("Load Plan",          self._load_plan,    "#21262d", 1, 1)
        action_btn("Export CSV",         self._export_csv,   "#21262d", 2, 0)
        action_btn("Clear All Zones",    self._clear_all,    "#b91c1c", 2, 1)

        # Tips
        tips_sec = section(right, "Tips")
        tk.Label(tips_sec,
                 text=("D / Draw Zone   → zone-draw mode\n"
                       "E / Erase Pixels→ pixel-erase mode\n"
                       "Left drag       → draw zone or erase\n"
                       "Click zone      → select it\n"
                       "Right-click     → context menu\n"
                       "↑ / ▲ button   → rotate rectangle +15°\n"
                       "↓ / ▼ button   → rotate rectangle −15°\n"
                       "Scroll wheel    → zoom on cursor\n"
                       "Middle drag     → pan\n"
                       "Del             → delete selected zone\n"
                       "Ctrl+Z          → undo last zone\n"
                       "Ctrl+Shift+Z    → undo map edit\n"
                       "Save Map…       → export edited .pgm"),
                 bg="#0d1117", fg="#484f58",
                 font=("Helvetica", 7), justify=tk.LEFT,
                 anchor=tk.W).pack(padx=8, pady=4, fill=tk.X)

    # ── Event binding ─────────────────────────────────────────────────────────

    def _bind_events(self):
        c = self.canvas
        c.bind("<ButtonPress-1>",   self._on_press)
        c.bind("<B1-Motion>",       self._on_drag)
        c.bind("<ButtonRelease-1>", self._on_release)
        c.bind("<Motion>",          self._on_move)
        c.bind("<Button-3>",        self._on_right_click)
        c.bind("<ButtonPress-2>",   self._on_pan_press)
        c.bind("<B2-Motion>",       self._on_pan_drag)
        c.bind("<MouseWheel>",      self._on_scroll)   # Windows / macOS
        c.bind("<Button-4>",        self._on_scroll)   # Linux scroll up
        c.bind("<Button-5>",        self._on_scroll)   # Linux scroll down

        self.root.bind("<Control-o>",       lambda _: self._open_map_dialog())
        self.root.bind("<Control-s>",       lambda _: self._save_plan())
        self.root.bind("<Control-z>",       lambda _: self._undo_last())
        self.root.bind("<Control-Z>",       lambda _: self._undo_map_edit())   # Ctrl+Shift+Z
        self.root.bind("<Delete>",          lambda _: self._delete_selected())
        self.root.bind("<e>",               lambda _: self._set_tool_mode("erase"))
        self.root.bind("<d>",               lambda _: self._set_tool_mode("draw"))
        self.root.bind("<Up>",              lambda _: self._rotate_rect(+15))
        self.root.bind("<Down>",            lambda _: self._rotate_rect(-15))

    # ── Map loading ───────────────────────────────────────────────────────────

    def _open_map_dialog(self):
        yaml_path = filedialog.askopenfilename(
            title="Select map.yaml",
            filetypes=[("YAML files", "*.yaml *.yml"), ("All", "*.*")],
        )
        if not yaml_path:
            return
        pgm_path = os.path.splitext(yaml_path)[0] + ".pgm"
        if not os.path.exists(pgm_path):
            pgm_path = filedialog.askopenfilename(
                title="Select matching map image (.pgm / .png)",
                filetypes=[("Images", "*.pgm *.png *.bmp"), ("All", "*.*")],
            )
        if pgm_path:
            self._load_map(pgm_path, yaml_path)

    def _load_map(self, pgm_path: str, yaml_path: str):
        try:
            with open(yaml_path) as f:
                y = yaml.safe_load(f)
            img = Image.open(pgm_path).convert("RGB")
            self.map_info = MapInfo(
                image_path=pgm_path,
                yaml_path=yaml_path,
                resolution=float(y.get("resolution", 0.05)),
                origin=list(y.get("origin", [0.0, 0.0, 0.0])),
                width=img.width,
                height=img.height,
                occupied_thresh=float(y.get("occupied_thresh", 0.65)),
                free_thresh=float(y.get("free_thresh", 0.196)),
            )
            self.original_image = img
            self.zones.clear()
            self.sequence.clear()
            self.selected_key = None
            self._refresh_zone_lb()
            self._refresh_seq_var()
            self._update_map_info_panel()
            self.root.after(50, self._fit_map)   # wait for canvas to resize
            self._set_status(
                f"Loaded: {os.path.basename(pgm_path)}  "
                f"{img.width}×{img.height} px  |  "
                f"{self.map_info.resolution} m/px  |  "
                f"Map size: {img.width * self.map_info.resolution:.1f}×"
                f"{img.height * self.map_info.resolution:.1f} m"
            )
        except Exception as exc:
            messagebox.showerror("Error loading map", str(exc))

    def _update_map_info_panel(self):
        mi = self.map_info
        if not mi:
            return
        info = (
            f"File   : {os.path.basename(mi.image_path)}\n"
            f"Size   : {mi.width} × {mi.height} px\n"
            f"Res    : {mi.resolution} m/px\n"
            f"Origin : ({mi.origin[0]:.3f}, {mi.origin[1]:.3f}) m\n"
            f"Extent : {mi.width*mi.resolution:.2f} × {mi.height*mi.resolution:.2f} m"
        )
        self._map_info_text.config(state=tk.NORMAL)
        self._map_info_text.delete("1.0", tk.END)
        self._map_info_text.insert("1.0", info)
        self._map_info_text.config(state=tk.DISABLED)

    # ── Zoom & pan ────────────────────────────────────────────────────────────

    def _fit_map(self):
        if not self.original_image:
            return
        cw = self.canvas.winfo_width()  or 800
        ch = self.canvas.winfo_height() or 600
        self.zoom   = min(cw / self.original_image.width,
                          ch / self.original_image.height, 4.0)
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._redraw()

    def _reset_zoom(self):
        self.zoom   = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._redraw()

    def _zoom_in(self):
        self._apply_zoom(self.zoom * 1.25)

    def _zoom_out(self):
        self._apply_zoom(self.zoom / 1.25)

    def _apply_zoom(self, new_zoom: float, cx: float = None, cy: float = None):
        """Set zoom, optionally anchoring to canvas point (cx, cy)."""
        old = self.zoom
        self.zoom = max(0.05, min(12.0, new_zoom))
        if cx is not None and cy is not None:
            scale = self.zoom / old
            self._pan_x = cx - scale * (cx - self._pan_x)
            self._pan_y = cy - scale * (cy - self._pan_y)
        self._zoom_label.config(text=f"{int(self.zoom * 100)}%")
        self._redraw()

    def _on_scroll(self, event):
        factor = 1.12
        if event.num == 5 or (hasattr(event, "delta") and event.delta < 0):
            factor = 1 / factor
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        self._apply_zoom(self.zoom * factor, cx, cy)

    def _on_pan_press(self, event):
        self._pan_anchor = (event.x, event.y, self._pan_x, self._pan_y)

    def _on_pan_drag(self, event):
        if self._pan_anchor:
            ax, ay, apx, apy = self._pan_anchor
            self._pan_x = apx + (event.x - ax)
            self._pan_y = apy + (event.y - ay)
            self._redraw()

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _canvas_to_pixel(self, cx: float, cy: float) -> Tuple[float, float]:
        return (cx - self._pan_x) / self.zoom, (cy - self._pan_y) / self.zoom

    def _pixel_to_canvas(self, px: float, py: float) -> Tuple[float, float]:
        return px * self.zoom + self._pan_x, py * self.zoom + self._pan_y

    # ── Mouse drawing ─────────────────────────────────────────────────────────

    def _on_press(self, event):
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        px, py = self._canvas_to_pixel(cx, cy)

        # In draw mode, clicking a zone selects it (erase mode ignores existing zones)
        if self._tool_mode == "draw":
            hit = self._zone_at(px, py)
            if hit:
                self._select(hit.key)
                return

        if not self.map_info:
            return

        # Start rubber-band rectangle (colour differs by mode)
        self._drawing = True
        self._draw_start_px = (px, py)
        scx, scy = self._pixel_to_canvas(px, py)
        if self._tool_mode == "erase":
            self._rubber_band_id = self.canvas.create_rectangle(
                scx, scy, cx, cy,
                outline="#FF8C00", fill="#FF8C00", stipple="gray12",
                width=2, dash=(6, 3))
        else:
            self._rubber_band_id = self.canvas.create_rectangle(
                scx, scy, cx, cy, outline="#FFFFFF", width=2, dash=(4, 4))
            self._select(None)

    def _on_drag(self, event):
        if not self._drawing or not self._draw_start_px:
            return
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        sx, sy = self._pixel_to_canvas(*self._draw_start_px)
        if self._rubber_band_id:
            self.canvas.coords(self._rubber_band_id, sx, sy, cx, cy)

    def _on_release(self, event):
        if not self._drawing:
            return
        self._drawing = False
        if self._rubber_band_id:
            self.canvas.delete(self._rubber_band_id)
            self._rubber_band_id = None

        if not self._draw_start_px or not self.map_info:
            return

        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        ex, ey = self._canvas_to_pixel(cx, cy)
        sx, sy = self._draw_start_px
        self._draw_start_px = None

        if (abs(ex - sx) * self.zoom < MIN_RECT_PX or
                abs(ey - sy) * self.zoom < MIN_RECT_PX):
            self._set_status("Rectangle too small — try again.")
            return

        # ── Erase mode: paint pixels white and return ─────────────────────────
        if self._tool_mode == "erase":
            self._erase_region(
                int(min(sx, ex)), int(min(sy, ey)),
                int(max(sx, ex)), int(max(sy, ey)),
            )
            return

        key   = self._next_key()
        color = ZONE_COLORS[len(self.zones) % len(ZONE_COLORS)]

        try:
            rw = float(self.robot_width_var.get())
        except ValueError:
            rw = DEFAULT_ROBOT_WIDTH

        zone = CoverageZone(
            key=key,
            x1=int(sx), y1=int(sy),
            x2=int(ex), y2=int(ey),
            path_type=self.path_type_var.get(),
            robot_width=rw,
            color=color,
        )

        # Auto-select angle: along the longer world dimension
        w_m = abs(zone.right - zone.left) * self.map_info.resolution
        h_m = abs(zone.bottom - zone.top) * self.map_info.resolution
        zone.angle = 0.0 if w_m >= h_m else 90.0

        self.zones[key] = zone
        self.sequence.append(key)
        self._refresh_zone_lb()
        self._refresh_seq_var()
        self._select(key)
        self._redraw()

        area_m2 = w_m * h_m
        n_stripes = max(1, int(math.ceil(min(w_m, h_m) / rw)))
        self._set_status(
            f"Zone '{key}' created  |  "
            f"{w_m:.2f}×{h_m:.2f} m  ({area_m2:.1f} m²)  |  "
            f"~{n_stripes} stripes at {rw} m spacing"
        )

    # ── Erase tool ────────────────────────────────────────────────────────────

    def _set_tool_mode(self, mode: str):
        """Switch between 'draw' and 'erase' tool modes."""
        self._tool_mode = mode
        if mode == "erase":
            self._btn_draw .config(bg="#21262d", fg="#aaa",  font=("Helvetica", 8))
            self._btn_erase.config(bg="#FF8C00", fg="white", font=("Helvetica", 8, "bold"))
            self.canvas.config(cursor="dotbox")
            self._set_status("ERASE mode — drag a rectangle to mark pixels as free.  "
                             "Press D or click Draw Zone to switch back.")
        else:
            self._btn_draw .config(bg="#1f6feb", fg="white", font=("Helvetica", 8, "bold"))
            self._btn_erase.config(bg="#21262d", fg="#aaa",  font=("Helvetica", 8))
            self.canvas.config(cursor="crosshair")
            self._set_status("DRAW mode — drag to create a new zone rectangle.")

    def _erase_region(self, px1: int, py1: int, px2: int, py2: int):
        """
        Paint a pixel rectangle white (free) on self.original_image,
        push the pre-edit image onto the undo stack first.
        Coordinates are in image-pixel space (may be outside image bounds — clipped).
        """
        if not self.original_image:
            return

        # Clamp to image bounds
        iw, ih = self.original_image.width, self.original_image.height
        x1 = max(0, min(px1, px2))
        y1 = max(0, min(py1, py2))
        x2 = min(iw, max(px1, px2))
        y2 = min(ih, max(py1, py2))
        if x2 <= x1 or y2 <= y1:
            return

        # Push undo snapshot (keep at most 20 levels)
        self._img_undo_stack.append(self.original_image.copy())
        if len(self._img_undo_stack) > 20:
            self._img_undo_stack.pop(0)

        # Paint the region white (free space = 255)
        from PIL import ImageDraw as _IDrawMod
        draw = _IDrawMod.Draw(self.original_image)
        draw.rectangle([x1, y1, x2, y2], fill=(255, 255, 255))
        del draw

        # Invalidate any zone waypoints that overlap the erased area
        # (their paths may now pass through newly-free space — user should regenerate)
        for z in self.zones.values():
            if not (z.right < x1 or z.left > x2 or z.bottom < y1 or z.top > y2):
                z.waypoints = []
        self._refresh_zone_lb()

        w_m = (x2 - x1) * self.map_info.resolution
        h_m = (y2 - y1) * self.map_info.resolution
        self._set_status(
            f"Erased {x2-x1}×{y2-y1} px  ({w_m:.2f}×{h_m:.2f} m)  "
            f"—  undo with Ctrl+Shift+Z  |  save map with 'Save Map…'"
        )
        self._redraw()

    def _undo_map_edit(self):
        """Restore the previous image from the undo stack."""
        if not self._img_undo_stack:
            self._set_status("Nothing to undo for map edits.")
            return
        self.original_image = self._img_undo_stack.pop()
        self._set_status(
            f"Map edit undone.  {len(self._img_undo_stack)} undo level(s) remaining.")
        self._redraw()

    def _save_map_dialog(self):
        """Save the (possibly edited) map image back to a .pgm file."""
        if not self.original_image:
            messagebox.showwarning("No map", "No map loaded.")
            return
        path = filedialog.asksaveasfilename(
            title="Save edited map image",
            initialfile="map_edited.pgm",
            defaultextension=".pgm",
            filetypes=[("PGM image", "*.pgm"), ("PNG image", "*.png"),
                       ("All files", "*.*")],
        )
        if not path:
            return
        # Save as grayscale
        self.original_image.convert("L").save(path)
        self._set_status(f"Map saved → {os.path.basename(path)}")

    # ── Cursor / status ───────────────────────────────────────────────────────

    def _on_move(self, event):
        if not self.map_info:
            return
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        px, py = self._canvas_to_pixel(cx, cy)
        wx, wy = self.map_info.pixel_to_world(px, py)
        self._cursor_var.set(
            f"pixel ({int(px)}, {int(py)})   world ({wx:.3f} m, {wy:.3f} m)"
        )

    def _on_right_click(self, event):
        cx = self.canvas.canvasx(event.x)
        cy = self.canvas.canvasy(event.y)
        px, py = self._canvas_to_pixel(cx, cy)
        zone = self._zone_at(px, py)
        if not zone:
            return
        self._select(zone.key)
        m = tk.Menu(self.root, tearoff=0)
        m.add_command(
            label=f"Zone '{zone.key}'  —  {zone.path_type}  ∠{zone.angle:.0f}°",
            state=tk.DISABLED)
        m.add_separator()
        m.add_command(label="Generate Path",   command=self._generate_selected)
        m.add_command(label="Delete Zone",     command=self._delete_selected)
        m.tk_popup(event.x_root, event.y_root)

    # ── Zone management ───────────────────────────────────────────────────────

    def _next_key(self) -> str:
        for i in range(26):
            k = chr(ord("a") + i)
            if k not in self.zones:
                return k
        for i in range(26):
            for j in range(26):
                k = chr(ord("a") + i) + chr(ord("a") + j)
                if k not in self.zones:
                    return k
        return f"z{len(self.zones)}"

    def _zone_at(self, px: float, py: float) -> Optional[CoverageZone]:
        for z in reversed(list(self.zones.values())):
            if z.contains_point(px, py):
                return z
        return None

    def _select(self, key: Optional[str]):
        self.selected_key = key
        if key and key in self.zones:
            z = self.zones[key]
            self.path_type_var.set(z.path_type)
            self.robot_width_var.set(f"{z.robot_width:.2f}")
            self._angle_dvar.set(z.angle)
            self.angle_str_var.set(f"{z.angle:.1f}")
            self._rect_angle_var.set(z.rect_angle)
            self.rect_angle_str_var.set(f"{z.rect_angle:.1f}")
            # Update listbox
            for i, k in enumerate(self.sequence):
                if k == key:
                    self._zone_lb.selection_clear(0, tk.END)
                    self._zone_lb.selection_set(i)
                    self._zone_lb.see(i)
                    break
            if z.waypoints:
                length = PathGenerator.path_length_m(z.waypoints)
                n_cov  = sum(1 for w in z.waypoints if w.get("type") == "coverage")
                self._path_len_var.set(
                    f"Path: {length:.1f} m  |  {n_cov} coverage pts")
            else:
                self._path_len_var.set("No path generated yet")
        else:
            self._path_len_var.set("No zone selected")
        self._redraw()

    def _on_lb_select(self, _event=None):
        sel = self._zone_lb.curselection()
        if sel and sel[0] < len(self.sequence):
            self._select(self.sequence[sel[0]])

    def _on_props_change(self, _event=None):
        if not self.selected_key or self.selected_key not in self.zones:
            return
        z = self.zones[self.selected_key]
        z.path_type = self.path_type_var.get()
        try:
            z.robot_width = max(0.01, float(self.robot_width_var.get()))
        except ValueError:
            pass
        z.waypoints = []
        self._path_len_var.set("No path generated yet")
        self._redraw()

    def _on_angle_entry_commit(self, _event=None):
        try:
            a = float(self.angle_str_var.get()) % 360
        except ValueError:
            return
        self._angle_dvar.set(a)
        if self.selected_key and self.selected_key in self.zones:
            self.zones[self.selected_key].angle = a
            self.zones[self.selected_key].waypoints = []
            self._path_len_var.set("No path generated yet")
            self._redraw()

    def _on_scale_moved(self, val):
        """Called when the angle scale is dragged."""
        a = float(val)
        self.angle_str_var.set(f"{a:.1f}")
        if self.selected_key and self.selected_key in self.zones:
            self.zones[self.selected_key].angle = a
            self.zones[self.selected_key].waypoints = []
            self._path_len_var.set("No path generated yet")
            self._redraw()

    def _sync_angle_entry_from_scale(self, *_args):
        """Keep text entry in sync when scale changes."""
        try:
            self.angle_str_var.set(f"{self._angle_dvar.get():.1f}")
        except tk.TclError:
            pass

    def _rotate_angle(self, delta: float):
        """Increment or decrement the stripe angle of the selected zone by delta degrees."""
        if not self.selected_key or self.selected_key not in self.zones:
            return
        z = self.zones[self.selected_key]
        a = (z.angle + delta) % 180
        z.angle = a
        z.waypoints = []
        self._angle_dvar.set(a)
        self.angle_str_var.set(f"{a:.1f}")
        self._path_len_var.set("No path generated yet")
        self._redraw()

    def _rotate_rect(self, delta: float):
        """Rotate the selected zone's rectangle by delta degrees."""
        if not self.selected_key or self.selected_key not in self.zones:
            return
        z = self.zones[self.selected_key]
        a = (z.rect_angle + delta) % 360
        z.rect_angle = a
        z.waypoints = []
        self._rect_angle_var.set(a)
        self.rect_angle_str_var.set(f"{a:.1f}")
        self._path_len_var.set("No path generated yet")
        self._redraw()

    def _on_rect_angle_commit(self, _event=None):
        try:
            a = float(self.rect_angle_str_var.get()) % 360
        except ValueError:
            return
        self._rect_angle_var.set(a)
        if self.selected_key and self.selected_key in self.zones:
            self.zones[self.selected_key].rect_angle = a
            self.zones[self.selected_key].waypoints = []
            self._path_len_var.set("No path generated yet")
            self._redraw()

    def _delete_selected(self):
        if not self.selected_key:
            return
        key = self.selected_key
        self.zones.pop(key, None)
        if key in self.sequence:
            self.sequence.remove(key)
        self.selected_key = None
        self._refresh_zone_lb()
        self._refresh_seq_var()
        self._redraw()

    def _undo_last(self):
        if not self.sequence:
            return
        key = self.sequence.pop()
        self.zones.pop(key, None)
        if self.selected_key == key:
            self.selected_key = None
        self._refresh_zone_lb()
        self._refresh_seq_var()
        self._redraw()

    def _clear_all(self):
        if not self.zones:
            return
        if messagebox.askyesno("Clear All", "Delete ALL zones?"):
            self.zones.clear()
            self.sequence.clear()
            self.selected_key = None
            self._refresh_zone_lb()
            self._refresh_seq_var()
            self._redraw()

    def _move_up(self):
        sel = self._zone_lb.curselection()
        if not sel or sel[0] == 0:
            return
        i = sel[0]
        self.sequence[i - 1], self.sequence[i] = self.sequence[i], self.sequence[i - 1]
        self._refresh_zone_lb()
        self._zone_lb.selection_set(i - 1)
        self._refresh_seq_var()

    def _move_down(self):
        sel = self._zone_lb.curselection()
        if not sel or sel[0] >= len(self.sequence) - 1:
            return
        i = sel[0]
        self.sequence[i], self.sequence[i + 1] = self.sequence[i + 1], self.sequence[i]
        self._refresh_zone_lb()
        self._zone_lb.selection_set(i + 1)
        self._refresh_seq_var()

    def _refresh_zone_lb(self):
        self._zone_lb.delete(0, tk.END)
        for key in self.sequence:
            z = self.zones.get(key)
            if not z:
                continue
            check = "✓" if z.waypoints else " "
            n_pts = len(z.waypoints) if z.waypoints else 0
            self._zone_lb.insert(
                tk.END,
                f" [{check}] {key}  {z.path_type}  "
                f"∠{z.angle:.0f}° R{z.rect_angle:.0f}°  {n_pts}pts",
            )

    def _refresh_seq_var(self):
        self._seq_var.set(",".join(self.sequence))

    def _active_sequence(self) -> List[str]:
        """Sequence from the entry widget (user may have edited it)."""
        raw = self._seq_var.get().strip()
        if raw:
            return [k.strip() for k in raw.split(",") if k.strip() in self.zones]
        return [k for k in self.sequence if k in self.zones]

    # ── Path generation ───────────────────────────────────────────────────────

    def _generate_all(self):
        if not self.map_info:
            messagebox.showwarning("No map", "Open a map first.")
            return
        total = 0
        for z in self.zones.values():
            z.waypoints = PathGenerator.generate(z, self.map_info)
            total += len(z.waypoints)
        self._refresh_zone_lb()
        self._redraw()
        if self.selected_key:
            self._select(self.selected_key)
        total_len = sum(PathGenerator.path_length_m(z.waypoints)
                        for z in self.zones.values() if z.waypoints)
        self._set_status(
            f"Generated paths for {len(self.zones)} zones  |  "
            f"{total} waypoints  |  total length ≈ {total_len:.1f} m"
        )

    def _generate_selected(self):
        if not self.selected_key or self.selected_key not in self.zones:
            messagebox.showwarning("No zone selected", "Click a zone first.")
            return
        z = self.zones[self.selected_key]
        z.waypoints = PathGenerator.generate(z, self.map_info)
        self._refresh_zone_lb()
        self._select(self.selected_key)   # refresh length label
        self._redraw()

    # ── Rendering ─────────────────────────────────────────────────────────────

    def _redraw(self):
        self.canvas.delete("all")
        if not self.original_image or not self.map_info:
            return

        # Scaled map image
        iw = max(1, int(self.original_image.width  * self.zoom))
        ih = max(1, int(self.original_image.height * self.zoom))
        scaled = self.original_image.resize((iw, ih), _RESAMPLE)
        self._tk_image = ImageTk.PhotoImage(scaled)
        self.canvas.create_image(self._pan_x, self._pan_y,
                                  anchor=tk.NW, image=self._tk_image)
        self.canvas.configure(scrollregion=(
            min(0, self._pan_x),
            min(0, self._pan_y),
            max(iw + self._pan_x, self.canvas.winfo_width()),
            max(ih + self._pan_y, self.canvas.winfo_height()),
        ))

        # Zones and paths
        for z in self.zones.values():
            self._draw_zone(z)
            if self.show_paths_var.get() and z.waypoints:
                self._draw_path(z)

    def _draw_zone(self, z: CoverageZone):
        # Compute rotated corners in canvas coords
        corners_c = [self._pixel_to_canvas(px, py) for px, py in z.corners_px()]
        pts = [coord for pt in corners_c for coord in pt]   # flat list for create_polygon
        is_sel = z.key == self.selected_key
        lw     = 3 if is_sel else 2

        # Translucent fill (stipple approximation)
        self.canvas.create_polygon(pts, fill=z.color, stipple="gray25", outline="")
        # Border
        self.canvas.create_polygon(pts, fill="",
                                    outline="#FFFFFF" if is_sel else z.color, width=lw)

        # Label — place near first corner (top-left when rect_angle ≈ 0)
        if self.show_numbers_var.get():
            fs = max(9, min(22, int(15 * self.zoom)))
            lx, ly = corners_c[0]   # TL corner
            self.canvas.create_text(lx + 1, ly + 1, text=z.key.upper(),
                                     anchor=tk.NW, font=("Helvetica", fs, "bold"), fill="#000")
            self.canvas.create_text(lx,     ly,     text=z.key.upper(),
                                     anchor=tk.NW, font=("Helvetica", fs, "bold"), fill="#FFF")

        # Stripe angle indicator — small arrow from centre
        cx_c = sum(pt[0] for pt in corners_c) / 4
        cy_c = sum(pt[1] for pt in corners_c) / 4
        if z.angle != 0.0:
            dx = corners_c[1][0] - corners_c[0][0]
            dy = corners_c[1][1] - corners_c[0][1]
            w_c = math.hypot(dx, dy)
            h_c = math.hypot(corners_c[3][0] - corners_c[0][0],
                              corners_c[3][1] - corners_c[0][1])
            r   = max(12, min(24, w_c / 4, h_c / 4))
            # stripe angle is in world/map coords; canvas y is inverted
            ar = math.radians(z.angle + z.rect_angle)
            ex = cx_c + r * math.cos(ar)
            ey = cy_c - r * math.sin(ar)
            self.canvas.create_line(cx_c, cy_c, ex, ey,
                                     fill=z.color, width=2, arrow=tk.LAST)

    def _draw_path(self, z: CoverageZone):
        if not self.map_info:
            return
        pts_canvas = []
        for wp in z.waypoints:
            px, py = self.map_info.world_to_pixel(wp["x"], wp["y"])
            pts_canvas.append(self._pixel_to_canvas(px, py))

        color = z.color
        dot_r = max(2, int(2.5 * self.zoom))

        # Group by stripe index
        by_stripe: Dict[int, List] = {}
        for i, wp in enumerate(z.waypoints):
            s = wp.get("stripe", 0)
            by_stripe.setdefault(s, []).append(i)

        # Draw stripe lines (solid) and inter-stripe transitions (dashed)
        for i in range(0, len(pts_canvas) - 1):
            wp_a = z.waypoints[i]
            wp_b = z.waypoints[i + 1]
            ca = pts_canvas[i]
            cb = pts_canvas[i + 1]
            same_stripe = wp_a.get("stripe") == wp_b.get("stripe")
            if same_stripe:
                self.canvas.create_line(ca[0], ca[1], cb[0], cb[1],
                                         fill=color, width=max(1.0, 1.5 * self.zoom))
            else:
                self.canvas.create_line(ca[0], ca[1], cb[0], cb[1],
                                         fill=color, width=1, dash=(4, 3))

        # Waypoint dots
        for i, (cx, cy) in enumerate(pts_canvas):
            wp = z.waypoints[i]
            if wp.get("type") == "transition":
                continue
            dot_fill = "#FFFFFF" if i == 0 else color
            self.canvas.create_oval(cx - dot_r, cy - dot_r,
                                     cx + dot_r, cy + dot_r,
                                     fill=dot_fill, outline="#000", width=1)

        # Start marker (larger white dot + arrow)
        if len(pts_canvas) >= 2:
            p0, p1 = pts_canvas[0], pts_canvas[1]
            r2 = dot_r + 2
            self.canvas.create_oval(p0[0] - r2, p0[1] - r2,
                                     p0[0] + r2, p0[1] + r2,
                                     fill="#FFF", outline=color, width=2)
            self.canvas.create_line(p0[0], p0[1], p1[0], p1[1],
                                     fill="#FFF", width=2, arrow=tk.LAST)

    # ── Save / load / export ──────────────────────────────────────────────────

    def _plan_dict(self) -> dict:
        mi = self.map_info
        return {
            "version": "1.0",
            "tool": "coverage_path_planner",
            "map": {
                "pgm_file":   os.path.basename(mi.image_path) if mi else "",
                "yaml_file":  os.path.basename(mi.yaml_path)  if mi else "",
                "resolution": mi.resolution   if mi else 0,
                "origin":     mi.origin       if mi else [0, 0, 0],
                "width_px":   mi.width        if mi else 0,
                "height_px":  mi.height       if mi else 0,
            },
            "sequence": self._active_sequence(),
            "zones": {k: z.to_dict(mi) for k, z in self.zones.items()},
        }

    def _save_plan(self):
        if not self.zones:
            messagebox.showwarning("Nothing to save", "Draw at least one zone.")
            return
        path = filedialog.asksaveasfilename(
            title="Save Coverage Plan",
            defaultextension=".json",
            filetypes=[("JSON", "*.json"), ("All", "*.*")],
        )
        if not path:
            return
        with open(path, "w") as f:
            json.dump(self._plan_dict(), f, indent=2)
        self._set_status(f"Plan saved → {os.path.basename(path)}")

    def _load_plan(self):
        path = filedialog.askopenfilename(
            title="Load Coverage Plan",
            filetypes=[("JSON", "*.json"), ("All", "*.*")],
        )
        if not path:
            return
        try:
            with open(path) as f:
                plan = json.load(f)
            self.zones.clear()
            self.sequence = plan.get("sequence", [])
            for key, zd in plan.get("zones", {}).items():
                pb = zd["pixel_bounds"]
                self.zones[key] = CoverageZone(
                    key=key,
                    x1=pb["x1"], y1=pb["y1"],
                    x2=pb["x2"], y2=pb["y2"],
                    path_type=zd.get("path_type", "boustrophedon"),
                    robot_width=zd.get("robot_width_m", DEFAULT_ROBOT_WIDTH),
                    angle=zd.get("angle_deg", 0.0),
                    rect_angle=zd.get("rect_angle_deg", 0.0),
                    color=zd.get("color", ZONE_COLORS[0]),
                    waypoints=zd.get("waypoints", []),
                )
            self.selected_key = None
            self._refresh_zone_lb()
            self._refresh_seq_var()
            self._redraw()
            self._set_status(f"Plan loaded: {len(self.zones)} zones from {os.path.basename(path)}")
        except Exception as exc:
            messagebox.showerror("Load error", str(exc))

    def _export_csv(self):
        any_wps = any(z.waypoints for z in self.zones.values())
        if not any_wps:
            messagebox.showwarning("No waypoints", "Generate paths first.")
            return
        path = filedialog.asksaveasfilename(
            title="Export Waypoints CSV",
            defaultextension=".csv",
            filetypes=[("CSV", "*.csv"), ("All", "*.*")],
        )
        if not path:
            return
        seq = self._active_sequence()
        rows = ["seq_order,zone_key,stripe,wp_index,type,x_m,y_m,yaw_rad"]
        for order, key in enumerate(seq):
            z = self.zones.get(key)
            if not z:
                continue
            for wi, wp in enumerate(z.waypoints):
                rows.append(
                    f"{order},{key},{wp.get('stripe',0)},{wi},"
                    f"{wp.get('type','coverage')},"
                    f"{wp['x']},{wp['y']},{wp.get('yaw', 0.0)}"
                )
        with open(path, "w") as f:
            f.write("\n".join(rows))
        self._set_status(f"CSV exported → {os.path.basename(path)}")

    # ── Misc ──────────────────────────────────────────────────────────────────

    def _set_status(self, msg: str):
        self._status_var.set(msg)


# ──────────────────────────────────────────────────────────────────────────────
# Entry point
# ──────────────────────────────────────────────────────────────────────────────

def _find_map_in_cwd():
    """Return (pgm_path, yaml_path) if a matching pair exists in cwd."""
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
            pgm = os.path.splitext(arg)[0] + ".pgm"
        elif arg.endswith(".pgm"):
            pgm = arg
            yml = os.path.splitext(arg)[0] + ".yaml"
    else:
        pgm, yml = _find_map_in_cwd()

    root = tk.Tk()
    root.configure(bg="#0d1117")
    CoveragePlannerApp(root, pgm, yml)
    root.mainloop()


if __name__ == "__main__":
    main()
