#!/usr/bin/env python3
"""Animate per-robot trajectories from a .pfg into an MP4 for slide use.

Reads ground-truth pose chains directly from the factor graph (one chain
per robot) and renders a short MP4 showing each robot's current position
plus a trailing segment of its recent path. A translucent plane at z=0
marks the sea surface. The camera orbits slowly so depth reads cleanly.

Edit the constants at the top of the file and run:

    cd pymantaray/localization
    uv run python animate_trajectories.py
"""

import os

import numpy as np
import pyvista as pv

from py_factor_graph.io.pyfg_text import read_from_pyfg_text


FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
OUTPUT_NAME = "floats_animation.mp4"

CLIP_SECONDS = 15.0
FPS = 30
TRAIL_SECONDS = 10 * 3600.0          # ~1 h sim-time trail behind each marker

MARKER_RADIUS_FRAC = 0.006      # relative to scene diagonal
TRAIL_LINE_WIDTH = 4
SURFACE_ALPHA = 0.18
SURFACE_COLOR = "lightskyblue"

ORBIT_DEGREES = 60.0            # total azimuth sweep over the clip
CAMERA_ELEVATION_DEG = 20.0
CAMERA_START_AZIMUTH_DEG = -120.0
CAMERA_DISTANCE_FRAC = 1.6      # multiple of scene diagonal

ROBOT_COLORS = ["royalblue", "firebrick", "darkgreen", "purple", "orange"]
WINDOW_SIZE = (1280, 720)


def _load_robot_data(file_path):
    """Return list of (xyz, timestamps) numpy arrays, one per robot."""
    fg = read_from_pyfg_text(file_path)
    robots = []
    for pose_chain in fg.pose_variables:
        if not pose_chain:
            continue
        pts = np.array([p.true_position for p in pose_chain], dtype=float)
        ts = np.array([p.timestamp for p in pose_chain], dtype=float)
        mask = np.all(np.isfinite(pts), axis=1) & np.isfinite(ts)
        pts, ts = pts[mask], ts[mask]
        if pts.shape[0] >= 2:
            order = np.argsort(ts)
            robots.append((pts[order], ts[order]))
    if not robots:
        raise RuntimeError(f"No usable pose chains in {file_path}")
    return robots


def _make_polyline(points):
    """Build a fresh PolyData line from an ordered (N, 3) point array."""
    n = points.shape[0]
    line = pv.PolyData(points.astype(np.float64, copy=True))
    cells = np.empty(n + 1, dtype=np.int64)
    cells[0] = n
    cells[1:] = np.arange(n, dtype=np.int64)
    line.lines = cells
    return line


def _camera_position(focal, distance, azimuth_deg, elevation_deg):
    """Camera xyz placed on a sphere around focal point. View-up is +z-down."""
    az = np.deg2rad(azimuth_deg)
    el = np.deg2rad(elevation_deg)
    offset = distance * np.array([
        np.cos(el) * np.cos(az),
        np.cos(el) * np.sin(az),
        np.sin(el),
    ])
    return tuple(focal + offset)


def main():
    out_path = os.path.join(os.path.dirname(FILE_PATH), OUTPUT_NAME)
    print(f"Reading {FILE_PATH} ...")
    robots = _load_robot_data(FILE_PATH)
    print(f"  {len(robots)} robot trajectories loaded")

    all_pts = np.vstack([pts for pts, _ in robots])
    bbox_min = all_pts.min(axis=0)
    bbox_max = all_pts.max(axis=0)
    bbox_min[2] = min(bbox_min[2], 0.0)            # include sea surface
    bbox_max[2] = max(bbox_max[2], 0.0)
    scene_center = 0.5 * (bbox_min + bbox_max)
    scene_diag = float(np.linalg.norm(bbox_max - bbox_min))
    marker_radius = max(1.0, MARKER_RADIUS_FRAC * scene_diag)
    cam_distance = CAMERA_DISTANCE_FRAC * scene_diag

    t_min = min(ts[0] for _, ts in robots)
    t_max = max(ts[-1] for _, ts in robots)
    n_frames = max(2, int(round(CLIP_SECONDS * FPS)))
    frame_times = np.linspace(t_min, t_max, n_frames)
    print(f"  sim window: {t_min:.0f}..{t_max:.0f} s  "
          f"({(t_max - t_min) / 3600:.1f} h)")
    print(f"  rendering {n_frames} frames at {FPS} fps  -> {out_path}")

    plotter = pv.Plotter(off_screen=True, window_size=WINDOW_SIZE)
    plotter.set_background("white")

    plane_extent = 1.4 * max(bbox_max[0] - bbox_min[0],
                             bbox_max[1] - bbox_min[1],
                             1.0)
    sea_surface = pv.Plane(
        center=(scene_center[0], scene_center[1], 0.0),
        direction=(0.0, 0.0, 1.0),
        i_size=plane_extent, j_size=plane_extent,
        i_resolution=1, j_resolution=1,
    )
    plotter.add_mesh(sea_surface, color=SURFACE_COLOR,
                     opacity=SURFACE_ALPHA, name="sea_surface",
                     show_edges=False, lighting=False)

    trail_actors = []
    marker_actors = []
    sphere_template = pv.Sphere(radius=marker_radius)
    template_pts = np.asarray(sphere_template.points, dtype=np.float64)
    for i, (pts, _) in enumerate(robots):
        color = ROBOT_COLORS[i % len(ROBOT_COLORS)]
        trail = _make_polyline(pts[:1].repeat(2, axis=0))
        trail_actor = plotter.add_mesh(
            trail, color=color, line_width=TRAIL_LINE_WIDTH,
            name=f"trail_{i}")
        trail_actors.append((trail, trail_actor))

        marker = pv.PolyData(template_pts + pts[0],
                             faces=sphere_template.faces)
        marker_actor = plotter.add_mesh(
            marker, color=color, name=f"marker_{i}",
            specular=0.3, smooth_shading=True)
        marker_actors.append(marker)

    plotter.show_axes()
    focal = scene_center
    plotter.camera.focal_point = tuple(focal)
    plotter.camera.position = _camera_position(
        focal, cam_distance, CAMERA_START_AZIMUTH_DEG, CAMERA_ELEVATION_DEG)
    plotter.camera.up = (0.0, 0.0, -1.0)
    plotter.reset_camera_clipping_range()

    plotter.open_movie(out_path, framerate=FPS)
    for k, t in enumerate(frame_times):
        azimuth = (CAMERA_START_AZIMUTH_DEG
                   + ORBIT_DEGREES * (k / max(1, n_frames - 1)))
        plotter.camera.position = _camera_position(
            focal, cam_distance, azimuth, CAMERA_ELEVATION_DEG)
        plotter.camera.focal_point = tuple(focal)
        plotter.camera.up = (0.0, 0.0, -1.0)
        plotter.reset_camera_clipping_range()

        for i, (pts, ts) in enumerate(robots):
            head_idx = int(np.searchsorted(ts, t, side="right") - 1)
            if head_idx < 0:
                continue
            tail_idx = int(np.searchsorted(ts, t - TRAIL_SECONDS, side="left"))
            tail_idx = min(tail_idx, head_idx)
            segment = pts[tail_idx:head_idx + 1]
            if segment.shape[0] < 2:
                segment = np.vstack([segment, segment[-1:]])
            trail_poly, _ = trail_actors[i]
            new_line = _make_polyline(segment)
            trail_poly.points = new_line.points
            trail_poly.lines = new_line.lines

            marker_poly = marker_actors[i]
            marker_poly.points = template_pts + pts[head_idx]

        plotter.write_frame()

    plotter.close()
    print(f"Wrote {out_path}")


if __name__ == "__main__":
    main()
