#!/usr/bin/env python3
"""Convex-hull helpers for landmark configurations.

Shared between the matplotlib APE plot (visualize_solver.py) and the
PyVista 3D views (vtk_plots.py) so both can speak the same notion of
"inside the LBL beacon hull" without duplicating geometry code.

The hull is **2D in the XY plane**: dive depth past the deepest beacon
doesn't give meaningful coverage information for LBL ranging, so the
geometric question is purely horizontal. For 3D rendering, the XY hull
polygon is extruded over the landmark Z range to produce a translucent
prism that visually anchors the beacon coverage column in the scene.

Both functions degrade silently when the hull is undefined (fewer than
three landmarks, collinear beacons, or empty query set). Callers can
treat a None hull or an all-False mask as "no hull-coverage info
available" and skip rendering.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from matplotlib.path import Path as MplPath


@dataclass
class LandmarkHull:
    """2D-XY convex hull of the landmark set, plus a 3D prism mesh
    suitable for VTK rendering.

    `xy_path` is the matplotlib Path of the hull polygon (closed) used
    for vectorized point-in-polygon tests. `mesh` is the same hull
    extruded over the landmark Z range as a pyvista PolyData; `None`
    when pyvista isn't importable.
    """
    xy_vertices: np.ndarray   # (M, 2) ordered hull vertices
    xy_path: MplPath
    mesh: object | None       # pv.PolyData, kept untyped to avoid eager import
    z_extent: tuple[float, float]


def build_landmark_hull(landmark_xyz: np.ndarray) -> LandmarkHull | None:
    """Compute the 2D convex hull of `landmark_xyz` (XY-projected) and
    extrude it into a 3D prism for visualization.

    Returns None if the hull is degenerate (fewer than 3 distinct points
    or all collinear in XY).
    """
    pts = np.asarray(landmark_xyz, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 3 or pts.shape[0] < 3:
        return None

    from scipy.spatial import ConvexHull, QhullError

    xy = pts[:, :2]
    try:
        hull = ConvexHull(xy)
    except QhullError:                    # collinear in XY
        return None

    # ConvexHull.vertices is CCW for 2D inputs; close the loop for the
    # matplotlib Path so it's an explicit polygon.
    hull_xy = xy[hull.vertices]
    closed = np.vstack([hull_xy, hull_xy[:1]])
    xy_path = MplPath(closed)

    z_min = float(pts[:, 2].min())
    z_max = float(pts[:, 2].max())
    if z_max <= z_min:                    # all beacons at same depth
        z_max = z_min + 1.0

    mesh = _extrude_prism(hull_xy, z_min, z_max)

    return LandmarkHull(xy_vertices=hull_xy,
                        xy_path=xy_path,
                        mesh=mesh,
                        z_extent=(z_min, z_max))


def inside_hull_mask(positions_xyz: np.ndarray,
                     hull: LandmarkHull | None) -> np.ndarray:
    """Per-point boolean mask: True where the position's XY lies inside
    the closed hull polygon. All-False if hull is None or positions_xyz
    is empty. The Z component of `positions_xyz` is ignored.
    """
    pts = np.asarray(positions_xyz, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0 or hull is None:
        return np.zeros(0 if pts.ndim != 2 else pts.shape[0], dtype=bool)
    return hull.xy_path.contains_points(pts[:, :2])


def _extrude_prism(hull_xy: np.ndarray, z_min: float, z_max: float):
    """Build a closed pyvista PolyData prism by extruding the 2D hull
    polygon between z_min and z_max. Returns None if pyvista isn't
    importable.
    """
    try:
        import pyvista as pv
    except ImportError:
        return None

    n = hull_xy.shape[0]
    if n < 3:
        return None

    base_pts = np.column_stack([hull_xy, np.full(n, z_min)])
    # One polygon face: [n_verts, v0, v1, ..., v_{n-1}].
    faces = np.concatenate([[n], np.arange(n)]).astype(np.int64)
    base = pv.PolyData(base_pts, faces=faces)
    prism = base.extrude((0.0, 0.0, z_max - z_min), capping=True)
    return prism.extract_surface(algorithm=None)
