#!/usr/bin/env python3
"""Convex-hull helpers for landmark configurations.

Used by the matplotlib APE plot (visualize_solver.py) to test whether
solve trajectories stay inside the LBL beacon footprint. The hull is
2D in the XY plane: dive depth past the deepest beacon doesn't give
meaningful coverage information for LBL ranging, so the geometric
question is purely horizontal.

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
    """2D-XY convex hull of the landmark set.

    `xy_path` is the matplotlib Path of the hull polygon (closed) used
    for vectorized point-in-polygon tests. `z_extent` records the depth
    span of the beacon set for callers that want to annotate the plot.
    """
    xy_vertices: np.ndarray   # (M, 2) ordered hull vertices
    xy_path: MplPath
    z_extent: tuple[float, float]


def build_landmark_hull(landmark_xyz: np.ndarray) -> LandmarkHull | None:
    """Compute the 2D convex hull of `landmark_xyz` (XY-projected).

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

    return LandmarkHull(xy_vertices=hull_xy,
                        xy_path=xy_path,
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
