#!/usr/bin/env python3
"""PyVista / VTK 3D views for the localization package.

One module owns all interactive 3D plotting so the stats / table code in
`debug_factor_graphs.py` and the range-diagnostic script in
`debug_factor_graph_ranges.py` share the same widgets, geometry helpers,
and colormap conventions.

Exposed functions:
  - plot_factors_in_world(solver, stats, ...)     — residual view
  - plot_leverage_in_world(solver, stats, ...)    — marginal leverage view
  - plot_range_errors_vtk(fg_data, ...)           — bellhop vs true range view
"""

import os

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np

from debug_factor_graphs import (
    FACTOR_TYPE_GPS_PRIOR,
    FACTOR_TYPE_LANDMARK_PRIOR,
    FACTOR_TYPE_BETWEEN,
    FACTOR_TYPE_RANGE_PP,
    FACTOR_TYPE_RANGE_PL,
    FACTOR_TYPE_RANGE_LL,
    FACTOR_TYPE_DEPTH,
    FactorRecord,
    FactorTypeStats,
)


# ───────────────────── shared geometry helpers ─────────────────────

def _lookup_position(values, solver, name: str) -> np.ndarray | None:
    """Return 3D position for a pose or landmark name, or None.

    Coordinates are returned unmodified. The PyVista viewer renders +z
    downward via a camera-up flip at the end of each plot, not by mutating
    the data.
    """
    key = solver.key_map.get(name)
    if key is None:
        return None
    pose_dict = solver.fg.pose_variables_dict
    landmark_dict = solver.fg.landmark_variables_dict
    try:
        if name in pose_dict:
            return np.asarray(values.atPose3(key).translation(), dtype=float)
        if name in landmark_dict:
            return np.asarray(values.atPoint3(key), dtype=float)
    except Exception:
        return None
    return None


def _factor_endpoints_xyz(solver, values, rec: FactorRecord
                          ) -> tuple[np.ndarray, np.ndarray] | None:
    """Return (p_a, p_b) in world coords for drawing a two-variable factor.
    Returns None for single-variable factors (priors) — those are rendered
    as glyphs in plot_factors_in_world, not as line segments.
    """
    if len(rec.endpoints) < 2:
        return None
    pts = [_lookup_position(values, solver, n) for n in rec.endpoints]
    if any(p is None for p in pts):
        return None
    return pts[0], pts[1]


def _build_line_poly(pa: np.ndarray, pb: np.ndarray):
    """Build a PyVista PolyData whose cells are independent line segments
    from pa[i] → pb[i]. pyvista is imported lazily so this helper can live
    at module scope without forcing pv at import time.
    """
    import pyvista as pv
    n = len(pa)
    pts = np.vstack([pa, pb])
    cells = np.column_stack([
        np.full(n, 2, dtype=np.int64),
        np.arange(n, dtype=np.int64),
        np.arange(n, 2 * n, dtype=np.int64),
    ]).ravel()
    return pv.PolyData(pts, lines=cells)


# Single-variable factor types are rendered as glyphs, not lines.
_PRIOR_TYPE_LABELS = frozenset({
    FACTOR_TYPE_GPS_PRIOR,
    FACTOR_TYPE_LANDMARK_PRIOR,
    FACTOR_TYPE_DEPTH,
})

# Line widths (pixels). Trajectories are deliberately 2× the ranging lines so
# the robot paths read as a clear backbone behind the dense range bundles.
_RANGE_LINE_WIDTH = 6
_TRAJ_LINE_WIDTH = 2 * _RANGE_LINE_WIDTH


# ───────────────────── residual view ─────────────────────

def plot_factors_in_world(solver,
                          stats: dict[str, FactorTypeStats],
                          save_dir: str | None = None,
                          prefix: str = "",
                          label_k: int = 50,
                          show: bool = True) -> None:
    """Interactive PyVista view of the solved factor graph.

    Each factor type is one PolyData trace (bulk line rendering). Cell
    scalars = whitened error so the worst segments stand out via colormap.
    The top `label_k` worst factors get persistent on-scene labels showing
    their residuals. Per-type checkbox toggles hide/show layers.
    """
    try:
        import pyvista as pv
    except ImportError:
        print("[vtk_plots] pyvista not installed — skipping 3D factor view. "
              "Install with `pip install pyvista`.")
        return
    if solver.result is None:
        print("[vtk_plots] solver.result is None — call .solve() first.")
        return

    values = solver.result
    plotter = pv.Plotter(title=f"Factor graph residuals — {prefix}")

    # --- collect trajectory + landmark points (needed for scene scale) ---
    robot_colors = ["royalblue", "firebrick", "darkgreen", "purple", "orange"]
    traj_point_sets: list[tuple[int, np.ndarray]] = []
    for i, pose_chain in enumerate(solver.fg.pose_variables):
        if not pose_chain:
            continue
        pts = np.array([_lookup_position(values, solver, p.name)
                        for p in pose_chain], dtype=float)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts):
            traj_point_sets.append((i, pts))

    lm_pts = np.empty((0, 3), dtype=float)
    if solver.fg.landmark_variables:
        lm_pts = np.array([_lookup_position(values, solver, lm.name)
                           for lm in solver.fg.landmark_variables], dtype=float)
        lm_pts = lm_pts[np.all(np.isfinite(lm_pts), axis=1)]

    # --- scene-scale glyph radius ---
    scale_stack = [pts for _, pts in traj_point_sets]
    if len(lm_pts):
        scale_stack.append(lm_pts)
    if scale_stack:
        all_world_pts = np.vstack(scale_stack)
        scene_diag = float(np.linalg.norm(
            all_world_pts.max(axis=0) - all_world_pts.min(axis=0)))
    else:
        scene_diag = 0.0
    glyph_radius = max(1.0, 0.005 * scene_diag)

    # --- trajectories ---
    for i, pts in traj_point_sets:
        if len(pts) >= 2:
            line = pv.lines_from_points(pts)
            plotter.add_mesh(line, color=robot_colors[i % len(robot_colors)],
                             line_width=_TRAJ_LINE_WIDTH, name=f"traj_{i}")

    # --- landmark variables (scene-scaled spheres, drawn on top of trajectories) ---
    if len(lm_pts):
        landmarks_poly = pv.PolyData(lm_pts).glyph(
            geom=pv.Sphere(radius=glyph_radius),
            orient=False, scale=False)
        plotter.add_mesh(landmarks_poly, color="gold", name="landmarks")

    # --- factors per type (colored by type, not residual) ---
    type_color_map = {
        FACTOR_TYPE_GPS_PRIOR:      "deepskyblue",
        FACTOR_TYPE_LANDMARK_PRIOR: "goldenrod",
        FACTOR_TYPE_BETWEEN:        "limegreen",
        FACTOR_TYPE_RANGE_PP:       "mediumorchid",
        FACTOR_TYPE_RANGE_PL:       "tomato",
        FACTOR_TYPE_RANGE_LL:       "saddlebrown",
        FACTOR_TYPE_DEPTH:          "slategray",
    }
    fallback_palette = ["cyan", "magenta", "yellow", "pink", "lime"]
    fallback_idx = 0

    type_actors: dict[str, object] = {}
    type_colors: dict[str, str] = {}

    # Range-factor types are filterable by residual threshold via a slider.
    range_types = {FACTOR_TYPE_RANGE_PP, FACTOR_TYPE_RANGE_PL,
                   FACTOR_TYPE_RANGE_LL}
    range_data: dict[str, dict] = {}

    for type_label, s in stats.items():
        # Priors are shown in plot_prior_health (a separate 2D panel) rather
        # than as 3D glyphs, which clutter the scene at scale.
        if type_label in _PRIOR_TYPE_LABELS:
            continue

        color = type_color_map.get(type_label)
        if color is None:
            color = fallback_palette[fallback_idx % len(fallback_palette)]
            fallback_idx += 1
        type_colors[type_label] = color

        # Two-variable factors → bulk line segments.
        points_a = []
        points_b = []
        raw_vals = []
        whi_vals = []
        for rec in s.records:
            endpoints = _factor_endpoints_xyz(solver, values, rec)
            if endpoints is None:
                continue
            points_a.append(endpoints[0])
            points_b.append(endpoints[1])
            raw_vals.append(rec.raw)
            whi_vals.append(rec.whitened)
        if not points_a:
            continue

        pa = np.asarray(points_a, dtype=float)
        pb = np.asarray(points_b, dtype=float)
        poly = _build_line_poly(pa, pb)

        actor = plotter.add_mesh(poly, color=color,
                                 line_width=_RANGE_LINE_WIDTH,
                                 name=f"factors_{type_label}")
        type_actors[type_label] = actor

        if type_label in range_types:
            range_data[type_label] = {
                "pa": pa,
                "pb": pb,
                "raw": np.asarray(raw_vals, dtype=float),
                "whitened": np.asarray(whi_vals, dtype=float),
                "actor": actor,
            }

    # --- labels for top-k worst ---
    all_records: list[FactorRecord] = []
    for s in stats.values():
        all_records.extend(s.records)
    all_records.sort(key=lambda r: r.whitened, reverse=True)
    top = all_records[:label_k]

    midpoints = []
    labels = []
    for rec in top:
        endpoints = _factor_endpoints_xyz(solver, values, rec)
        if endpoints is None:
            continue
        mid = 0.5 * (endpoints[0] + endpoints[1])
        midpoints.append(mid)
        ep_str = "↔".join(rec.endpoints) if len(rec.endpoints) > 1 \
            else rec.endpoints[0]
        labels.append(
            f"{ep_str}\nraw={rec.raw:.2f}  whitened={rec.whitened:.2f}")
    if midpoints:
        plotter.add_point_labels(
            np.asarray(midpoints, dtype=float), labels,
            font_size=28, text_color="black", shape="rounded_rect",
            shape_color="white", shape_opacity=0.85,
            bold=True,
            always_visible=True, point_size=0, name="worst_labels")

    # --- per-type visibility toggles + color legend ---
    row_h = 40
    btn_size = 28
    for idx, (type_label, actor) in enumerate(type_actors.items()):
        def _toggle(state, a=actor):
            a.SetVisibility(state)
        plotter.add_checkbox_button_widget(
            _toggle, value=True, position=(10, 10 + idx * row_h),
            size=btn_size, border_size=2,
            color_on=type_colors.get(type_label, "white"),
            color_off="gray")
        plotter.add_text(f" {type_label}",
                         position=(10 + btn_size + 8, 10 + idx * row_h),
                         font_size=20, color="black")

    # --- range-factor residual threshold slider + metric toggle ---
    if range_data:
        state = {"metric": "whitened", "threshold": 0.0}

        def _metric_max() -> float:
            vals = [d[state["metric"]] for d in range_data.values()]
            stacked = np.concatenate(vals) if vals else np.array([0.0])
            stacked = stacked[np.isfinite(stacked)]
            return float(max(1e-6, stacked.max() if stacked.size else 1.0))

        def _apply_threshold() -> None:
            for d in range_data.values():
                metric = d[state["metric"]]
                mask = metric >= state["threshold"]
                n = int(mask.sum())
                actor = d["actor"]
                if n == 0:
                    actor.SetVisibility(False)
                    continue
                new_poly = _build_line_poly(d["pa"][mask], d["pb"][mask])
                actor.GetMapper().SetInputData(new_poly)
                actor.SetVisibility(True)
            plotter.render()

        slider_ref: dict[str, object] = {}

        def _on_slider(value):
            state["threshold"] = float(value)
            _apply_threshold()

        def _slider_title() -> str:
            return ("min whitened residual" if state["metric"] == "whitened"
                    else "min raw residual (m)")

        slider_ref["widget"] = plotter.add_slider_widget(
            _on_slider,
            rng=[0.0, _metric_max()],
            value=0.0,
            title=_slider_title(),
            pointa=(0.25, 0.08),
            pointb=(0.75, 0.08),
            style="modern",
        )

        toggle_y = 10 + len(type_actors) * row_h + 10

        def _on_metric_toggle(raw_mode: bool) -> None:
            state["metric"] = "raw" if raw_mode else "whitened"
            state["threshold"] = 0.0
            new_max = _metric_max()
            # Rebuild the slider so title + range both refresh cleanly.
            plotter.clear_slider_widgets()
            slider_ref["widget"] = plotter.add_slider_widget(
                _on_slider,
                rng=[0.0, new_max],
                value=0.0,
                title=_slider_title(),
                pointa=(0.25, 0.08),
                pointb=(0.75, 0.08),
                style="modern",
            )
            _apply_threshold()

        plotter.add_checkbox_button_widget(
            _on_metric_toggle, value=False,
            position=(10, toggle_y), size=btn_size, border_size=2,
            color_on="orange", color_off="steelblue")
        plotter.add_text(" metric: raw (on) / whitened (off)",
                         position=(10 + btn_size + 8, toggle_y),
                         font_size=18, color="black")

    plotter.add_axes(label_size=(0.08, 0.08))
    plotter.show_grid(font_size=20)

    # Initial view with +z appearing downward on screen (engineering
    # depth-down convention). Camera-only — no data or scale is modified,
    # so label anchors and the axis widget stay correct.
    plotter.view_xz()
    plotter.camera.up = (0.0, 0.0, -1.0)
    plotter.reset_camera()

    if save_dir:
        html_path = os.path.join(save_dir, f"{prefix}_factors_3d.html")
        try:
            plotter.export_html(html_path)
            print(f"Saved {html_path}")
        except Exception as e:
            print(f"[vtk_plots] export_html failed: {e}")

    if show:
        plotter.show()
    else:
        plotter.close()


# ───────────────────── leverage view ─────────────────────

def plot_leverage_in_world(solver,
                           stats: dict[str, FactorTypeStats],
                           save_dir: str | None = None,
                           prefix: str = "",
                           label_k: int = 50,
                           show: bool = True) -> None:
    """Second PyVista window: range factors colored by marginal leverage.

    Requires `rec.leverage` to be populated (via `_attach_leverages_to_stats`).
    Shows only range factor types; between/odom drawn solid for context.
    Slider filters by leverage ≥ threshold.
    """
    try:
        import pyvista as pv
    except ImportError:
        print("[vtk_plots] pyvista not installed — skipping leverage 3D view.")
        return
    if solver.result is None:
        return

    values = solver.result
    plotter = pv.Plotter(title=f"Marginal leverage — {prefix}")

    # Trajectories + landmarks (shared with residual view for context).
    robot_colors = ["royalblue", "firebrick", "darkgreen", "purple", "orange"]
    traj_pts_all: list[np.ndarray] = []
    for i, pose_chain in enumerate(solver.fg.pose_variables):
        if not pose_chain:
            continue
        pts = np.array([_lookup_position(values, solver, p.name)
                        for p in pose_chain], dtype=float)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) >= 2:
            line = pv.lines_from_points(pts)
            plotter.add_mesh(line, color=robot_colors[i % len(robot_colors)],
                             line_width=_TRAJ_LINE_WIDTH, name=f"traj_{i}")
            traj_pts_all.append(pts)

    lm_pts = np.empty((0, 3), dtype=float)
    if solver.fg.landmark_variables:
        lm_pts = np.array([_lookup_position(values, solver, lm.name)
                           for lm in solver.fg.landmark_variables], dtype=float)
        lm_pts = lm_pts[np.all(np.isfinite(lm_pts), axis=1)]

    scale_stack = list(traj_pts_all)
    if len(lm_pts):
        scale_stack.append(lm_pts)
    if scale_stack:
        all_pts = np.vstack(scale_stack)
        scene_diag = float(np.linalg.norm(
            all_pts.max(axis=0) - all_pts.min(axis=0)))
    else:
        scene_diag = 0.0
    glyph_radius = max(1.0, 0.005 * scene_diag)

    if len(lm_pts):
        lm_mesh = pv.PolyData(lm_pts).glyph(
            geom=pv.Sphere(radius=glyph_radius), orient=False, scale=False)
        plotter.add_mesh(lm_mesh, color="gold", name="landmarks")

    # Range factors colored by leverage scalar. Use a truncated Reds
    # colormap so the low end is a muted pink (still visible) instead of
    # pure white, while the high end stays deep red for emphasis.
    leverage_cmap = ListedColormap(plt.cm.Reds(np.linspace(0.3, 1.0, 256)))

    range_types = {FACTOR_TYPE_RANGE_PP, FACTOR_TYPE_RANGE_PL,
                   FACTOR_TYPE_RANGE_LL}
    range_data: dict[str, dict] = {}

    for type_label, s in stats.items():
        if type_label not in range_types:
            continue
        points_a, points_b, lev_vals = [], [], []
        for rec in s.records:
            endpoints = _factor_endpoints_xyz(solver, values, rec)
            if endpoints is None or not np.isfinite(rec.leverage):
                continue
            points_a.append(endpoints[0])
            points_b.append(endpoints[1])
            lev_vals.append(rec.leverage)
        if not points_a:
            continue
        pa = np.asarray(points_a, dtype=float)
        pb = np.asarray(points_b, dtype=float)
        lev = np.asarray(lev_vals, dtype=float)

        poly = _build_line_poly(pa, pb)
        poly.cell_data["leverage"] = lev

        # Single-tone colormap (pale → saturated red) + opacity that
        # ramps from a floor (so low-leverage lines are still visible as
        # faint context) up to fully opaque for high-leverage outliers.
        # Single tone keeps the eye anchored on the bright end only —
        # unlike turbo/inferno which pop at both extremes.
        actor = plotter.add_mesh(
            poly, scalars="leverage", cmap=leverage_cmap,
            opacity=[0.25, 1.0], line_width=_RANGE_LINE_WIDTH,
            name=f"lev_{type_label}",
            scalar_bar_args={
                "title": f"{type_label} leverage",
                "vertical": True,
                "position_x": 0.88,
                "position_y": 0.10,
                "width": 0.08,
                "height": 0.7,
                "title_font_size": 22,
                "label_font_size": 20,
                "n_labels": 5,
                "fmt": "%.3g",
            })
        range_data[type_label] = {"pa": pa, "pb": pb, "lev": lev,
                                  "actor": actor}

    # Top-K leverage labels.
    all_recs: list[FactorRecord] = []
    for s in stats.values():
        if s.type_label in range_types:
            all_recs.extend(r for r in s.records if np.isfinite(r.leverage))
    all_recs.sort(key=lambda r: r.leverage, reverse=True)
    midpoints, labels = [], []
    for rec in all_recs[:label_k]:
        endpoints = _factor_endpoints_xyz(solver, values, rec)
        if endpoints is None:
            continue
        mid = 0.5 * (endpoints[0] + endpoints[1])
        midpoints.append(mid)
        ep_str = "↔".join(rec.endpoints)
        labels.append(f"{ep_str}\nlev={rec.leverage:.3g}")
    if midpoints:
        plotter.add_point_labels(
            np.asarray(midpoints, dtype=float), labels,
            font_size=24, text_color="black", shape="rounded_rect",
            shape_color="white", shape_opacity=0.85, bold=True,
            always_visible=True, point_size=0, name="lev_labels")

    # Slider on leverage threshold.
    if range_data:
        all_lev = np.concatenate([d["lev"] for d in range_data.values()])
        lev_max = float(max(1e-6, all_lev.max())) if all_lev.size else 1.0
        state = {"threshold": 0.0}

        def _apply(_=None):
            for d in range_data.values():
                mask = d["lev"] >= state["threshold"]
                n = int(mask.sum())
                actor = d["actor"]
                if n == 0:
                    actor.SetVisibility(False)
                    continue
                new_poly = _build_line_poly(d["pa"][mask], d["pb"][mask])
                new_poly.cell_data["leverage"] = d["lev"][mask]
                actor.GetMapper().SetInputData(new_poly)
                actor.SetVisibility(True)
            plotter.render()

        def _on_slider(value):
            state["threshold"] = float(value)
            _apply()

        plotter.add_slider_widget(
            _on_slider, rng=[0.0, lev_max], value=0.0,
            title="min leverage", pointa=(0.25, 0.08),
            pointb=(0.75, 0.08), style="modern")

    plotter.add_axes(label_size=(0.08, 0.08))
    plotter.show_grid(font_size=20)
    plotter.view_xz()
    plotter.camera.up = (0.0, 0.0, -1.0)
    plotter.reset_camera()

    if save_dir:
        html_path = os.path.join(save_dir, f"{prefix}_leverage_3d.html")
        try:
            plotter.export_html(html_path)
            print(f"Saved {html_path}")
        except Exception as e:
            print(f"[vtk_plots] export_html failed: {e}")

    if show:
        plotter.show()
    else:
        plotter.close()


# ───────────────────── range-error view (solver-less) ─────────────────────

def _pose_xyz(pose) -> np.ndarray:
    """Extract a pose's ground-truth 3D position as a numpy array."""
    return np.array([pose.true_x, pose.true_y, pose.true_z], dtype=float)


def _landmark_xyz(landmark) -> np.ndarray:
    """Extract a landmark's ground-truth 3D position as a numpy array."""
    return np.asarray(landmark.true_position, dtype=float)


def _resolve_xyz(name: str, pose_dict, landmark_dict) -> np.ndarray | None:
    """Ground-truth position for a variable name, or None if unknown."""
    if name in pose_dict:
        return _pose_xyz(pose_dict[name])
    if name in landmark_dict:
        return _landmark_xyz(landmark_dict[name])
    return None


def plot_range_errors_vtk(fg_data,
                          save_dir: str | None = None,
                          show: bool = True) -> None:
    """PyVista view of every range measurement colored by |measured - true|.

    Lines are split into two bundles (robot↔robot and robot↔landmark) so
    each can be toggled independently. A slider hides lines below a
    |error| threshold; four checkboxes toggle the two range bundles plus
    the robot trajectories and landmark glyphs.

    Operates on a raw FactorGraphData — no GTSAM solver required. Uses
    ground-truth positions (`pose.true_x/y/z`, `landmark.true_position`)
    so the scene reflects the simulated truth, not a solved estimate.
    """
    try:
        import pyvista as pv
    except ImportError:
        print("[vtk_plots] pyvista not installed — skipping range-error view.")
        return
    if not fg_data.range_measurements:
        print("[vtk_plots] no range measurements; skipping range-error view.")
        return

    # Lazy import to avoid pulling py_factor_graph at module load time.
    from py_factor_graph.modifiers import make_all_ranges_perfect
    true_fg = make_all_ranges_perfect(fg_data)

    pose_dict = fg_data.pose_variables_dict
    landmark_dict = fg_data.landmark_variables_dict

    rr_a, rr_b, rr_err = [], [], []
    rl_a, rl_b, rl_err = [], [], []
    skipped = 0
    for meas, true_meas in zip(fg_data.range_measurements,
                               true_fg.range_measurements):
        name_a, name_b = meas.association
        p_a = _resolve_xyz(name_a, pose_dict, landmark_dict)
        p_b = _resolve_xyz(name_b, pose_dict, landmark_dict)
        if p_a is None or p_b is None:
            skipped += 1
            continue
        abs_err = abs(meas.dist - true_meas.dist)
        a_is_pose = name_a in pose_dict
        b_is_pose = name_b in pose_dict
        if a_is_pose and b_is_pose:
            rr_a.append(p_a); rr_b.append(p_b); rr_err.append(abs_err)
        elif a_is_pose or b_is_pose:
            rl_a.append(p_a); rl_b.append(p_b); rl_err.append(abs_err)
        else:
            skipped += 1
    if skipped:
        print(f"[vtk_plots] skipped {skipped} unresolved / ambiguous ranges.")

    rr_a = np.asarray(rr_a, dtype=float) if rr_a else np.empty((0, 3))
    rr_b = np.asarray(rr_b, dtype=float) if rr_b else np.empty((0, 3))
    rr_err = np.asarray(rr_err, dtype=float)
    rl_a = np.asarray(rl_a, dtype=float) if rl_a else np.empty((0, 3))
    rl_b = np.asarray(rl_b, dtype=float) if rl_b else np.empty((0, 3))
    rl_err = np.asarray(rl_err, dtype=float)

    if rr_err.size == 0 and rl_err.size == 0:
        print("[vtk_plots] no resolvable range measurements; skipping view.")
        return

    all_err = np.concatenate([e for e in (rr_err, rl_err) if e.size])
    err_max = float(max(1e-6, all_err.max()))
    clim = [0.0, err_max]

    plotter = pv.Plotter(title="Range error |measured - true|")

    # --- trajectories (ground-truth) ---
    robot_colors = ["royalblue", "firebrick", "darkgreen", "purple", "orange"]
    traj_actors: list[object] = []
    traj_pts_all: list[np.ndarray] = []
    for i, pose_chain in enumerate(fg_data.pose_variables):
        if not pose_chain:
            continue
        pts = np.array([_pose_xyz(p) for p in pose_chain], dtype=float)
        pts = pts[np.all(np.isfinite(pts), axis=1)]
        if len(pts) >= 2:
            line = pv.lines_from_points(pts)
            actor = plotter.add_mesh(
                line, color=robot_colors[i % len(robot_colors)],
                line_width=_TRAJ_LINE_WIDTH, name=f"traj_{i}")
            traj_actors.append(actor)
            traj_pts_all.append(pts)

    # --- landmarks ---
    lm_pts = np.array([_landmark_xyz(lm) for lm in fg_data.landmark_variables],
                      dtype=float) if fg_data.landmark_variables else np.empty((0, 3))
    if lm_pts.size:
        lm_pts = lm_pts[np.all(np.isfinite(lm_pts), axis=1)]

    # Scene scale for sphere radius.
    scale_stack = list(traj_pts_all)
    if lm_pts.size:
        scale_stack.append(lm_pts)
    if scale_stack:
        all_world = np.vstack(scale_stack)
        scene_diag = float(np.linalg.norm(
            all_world.max(axis=0) - all_world.min(axis=0)))
    else:
        scene_diag = 0.0
    glyph_radius = max(1.0, 0.005 * scene_diag)

    lm_actor = None
    if lm_pts.size:
        lm_mesh = pv.PolyData(lm_pts).glyph(
            geom=pv.Sphere(radius=glyph_radius), orient=False, scale=False)
        lm_actor = plotter.add_mesh(lm_mesh, color="gold", name="landmarks")

    # --- range line bundles (colored by |error|) ---
    # Turbo: perceptually ordered, starts blue (low error) and ends red
    # (high error) so the extremes are legible in a presentation still.
    err_cmap = "turbo"

    bundles: dict[str, dict] = {}

    def _add_bundle(key: str, pa: np.ndarray, pb: np.ndarray,
                    err: np.ndarray, bar_title: str, bar_y: float) -> None:
        if pa.shape[0] == 0:
            return
        poly = _build_line_poly(pa, pb)
        poly.cell_data["abs_err"] = err
        actor = plotter.add_mesh(
            poly, scalars="abs_err", cmap=err_cmap, clim=clim,
            opacity=[0.25, 1.0], line_width=_RANGE_LINE_WIDTH, name=f"rng_{key}",
            scalar_bar_args={
                "title": bar_title,
                "vertical": True,
                "position_x": 0.88,
                "position_y": bar_y,
                "width": 0.08,
                "height": 0.35,
                "title_font_size": 24,
                "label_font_size": 22,
                "n_labels": 5,
                "fmt": "%.3g",
            })
        bundles[key] = {"pa": pa, "pb": pb, "err": err, "actor": actor}

    _add_bundle("rr", rr_a, rr_b, rr_err, "|err| rr (m)", 0.55)
    _add_bundle("rl", rl_a, rl_b, rl_err, "|err| rl (m)", 0.10)

    # --- slider on |error| threshold ---
    state = {"threshold": 0.0}

    def _apply(_=None):
        for d in bundles.values():
            mask = d["err"] >= state["threshold"]
            n = int(mask.sum())
            actor = d["actor"]
            if n == 0:
                actor.SetVisibility(False)
                continue
            new_poly = _build_line_poly(d["pa"][mask], d["pb"][mask])
            new_poly.cell_data["abs_err"] = d["err"][mask]
            actor.GetMapper().SetInputData(new_poly)
            actor.SetVisibility(True)
        plotter.render()

    def _on_slider(value):
        state["threshold"] = float(value)
        _apply()

    plotter.add_slider_widget(
        _on_slider, rng=[0.0, err_max], value=0.0,
        title="min |range error| (m)",
        pointa=(0.25, 0.08), pointb=(0.75, 0.08), style="modern")

    # --- checkboxes: rr / rl / robots / landmarks ---
    row_h = 40
    btn_size = 28
    rr_state = {"on": True}
    rl_state = {"on": True}
    robots_state = {"on": True}
    lm_state = {"on": True}

    def _toggle_rr(on: bool):
        rr_state["on"] = on
        actor = bundles.get("rr", {}).get("actor")
        if actor is not None:
            actor.SetVisibility(on)
        plotter.render()

    def _toggle_rl(on: bool):
        rl_state["on"] = on
        actor = bundles.get("rl", {}).get("actor")
        if actor is not None:
            actor.SetVisibility(on)
        plotter.render()

    def _toggle_robots(on: bool):
        robots_state["on"] = on
        for a in traj_actors:
            a.SetVisibility(on)
        plotter.render()

    def _toggle_landmarks(on: bool):
        lm_state["on"] = on
        if lm_actor is not None:
            lm_actor.SetVisibility(on)
        plotter.render()

    entries = [
        ("Robot↔Robot ranges",     _toggle_rr,        "mediumorchid"),
        ("Robot↔Landmark ranges",  _toggle_rl,        "tomato"),
        ("Robots",                 _toggle_robots,    "royalblue"),
        ("Landmarks",              _toggle_landmarks, "gold"),
    ]
    for idx, (label_text, cb, color_on) in enumerate(entries):
        plotter.add_checkbox_button_widget(
            cb, value=True, position=(10, 10 + idx * row_h),
            size=btn_size, border_size=2,
            color_on=color_on, color_off="gray")
        plotter.add_text(f" {label_text}",
                         position=(10 + btn_size + 8, 10 + idx * row_h),
                         font_size=20, color="black")

    plotter.add_axes(label_size=(0.08, 0.08))
    plotter.show_grid(font_size=20)
    plotter.view_xz()
    plotter.camera.up = (0.0, 0.0, -1.0)
    plotter.reset_camera()

    if save_dir:
        png_path = os.path.join(save_dir, "range_errors_3d.png")
        try:
            plotter.screenshot(png_path)
            print(f"Saved {png_path}")
        except Exception as e:
            print(f"[vtk_plots] screenshot failed: {e}")

    if show:
        plotter.show()
    else:
        plotter.close()
