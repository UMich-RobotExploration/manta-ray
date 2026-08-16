#!/usr/bin/env python3
"""Visualization for FactorGraphSolver results.

Plots ground-truth, initial estimate, and optimized trajectories with APE
comparison using the evo library.
"""

from __future__ import annotations

import os
import textwrap

import matplotlib
from matplotlib.collections import LineCollection
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from evo.core import metrics
from evo.tools import plot as evo_plot

from py_factor_graph.modifiers import make_all_ranges_perfect

from hull_utils import build_landmark_hull, inside_hull_mask
from pyfg_to_gtsam import FactorGraphSolver, extract_trajectory


def _shade_mask_spans(ax, mask: np.ndarray, *,
                      color: str, alpha: float, label: str) -> None:
    """Shade contiguous True-runs in mask using ax.axvspan.

    Only the first run carries the legend label so the legend stays clean.
    No-op when mask is all False.
    """
    if mask.size == 0 or not mask.any():
        return
    edges = np.diff(mask.astype(np.int8), prepend=0, append=0)
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    for i, (s, e) in enumerate(zip(starts, ends)):
        ax.axvspan(s - 0.5, e + 0.5, color=color, alpha=alpha,
                   label=label if i == 0 else None, zorder=0)


def _mask_run_boundaries(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return (entry_indices, exit_indices) for each contiguous True-run.

    Empty arrays when the mask has no True values. Each entry index is
    the first True pose in a run; each exit index is the last True pose.
    """
    if mask.size == 0 or not mask.any():
        return np.empty(0, dtype=int), np.empty(0, dtype=int)
    edges = np.diff(mask.astype(np.int8), prepend=0, append=0)
    entries = np.where(edges == 1)[0]
    exits = np.where(edges == -1)[0] - 1
    return entries, exits


def _label_mask_boundaries(ax, mask: np.ndarray, *,
                           color: str = "darkgreen",
                           fontsize: int = 8) -> None:
    """Annotate entry / exit pose indices of each True-run on `ax`.

    Each transition gets a thin vertical tick spanning the axis and a
    text label rendered in axes-fraction coordinates near the top, so
    labels sit above APE curves regardless of y-axis scale.
    """
    entries, exits = _mask_run_boundaries(mask)
    if entries.size == 0:
        return
    bbox = dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1.0)
    for idx in entries:
        ax.axvline(idx, color=color, alpha=0.4, linewidth=0.8, linestyle=":")
        ax.text(idx, 0.98, f"→{idx}",
                transform=ax.get_xaxis_transform(),
                ha="left", va="top",
                fontsize=fontsize, color=color, bbox=bbox)
    for idx in exits:
        ax.axvline(idx, color=color, alpha=0.4, linewidth=0.8, linestyle=":")
        ax.text(idx, 0.98, f"{idx}←",
                transform=ax.get_xaxis_transform(),
                ha="right", va="top",
                fontsize=fontsize, color=color, bbox=bbox)


def _measurement_pose_indices(solver: FactorGraphSolver,
                              pose_chain) -> tuple[set[int], set[int]]:
    """Find pose indices that have range measurements or GPS priors.

    Returns (range_indices, gps_indices) as sets of integer pose indices.
    GPS priors are all pose priors except the first-pose prior per robot.
    """
    name_to_idx = {p.name: i for i, p in enumerate(pose_chain)}
    pose_names = set(name_to_idx.keys())

    range_indices = set()
    for rm in solver.fg.range_measurements:
        for name in rm.association:
            if name in pose_names:
                range_indices.add(name_to_idx[name])

    first_pose_names = set()
    for chain in solver.fg.pose_variables:
        if chain:
            first_pose_names.add(chain[0].name)

    gps_indices = set()
    for prior in solver.fg.pose_priors:
        if prior.name in pose_names and prior.name not in first_pose_names:
            gps_indices.add(name_to_idx[prior.name])

    return range_indices, gps_indices


def plot_ape_histogram(ape_opt: metrics.APE,
                       ape_odom: metrics.APE,
                       robot_char: str,
                       estimate_label: str,
                       save_path: str | None = None) -> None:
    """Histogram of APE values with mean/median/RMSE overlays.

    Sibling to the per-pose-index APE line plot. Shows the population shape
    of the estimated trajectory's error vs. odometry-only.
    """
    if len(ape_opt.error) < 2:
        print(f"[ape-dist] robot {robot_char}: too few poses, skipping histogram")
        return

    estimate_legend = f"Estimated Traj w/ {estimate_label}"
    stats = ape_opt.get_all_statistics()
    mean_v = stats["mean"]
    median_v = stats["median"]
    rmse_v = stats["rmse"]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.hist(ape_odom.error, bins=50, alpha=0.45, color="tab:orange",
            label="Odometry Only Trajectory")
    ax.hist(ape_opt.error, bins=50, alpha=0.65, color="tab:blue",
            label=estimate_legend)
    ax.axvline(mean_v, color="black", linestyle="-", linewidth=1.2,
               label=f"mean = {mean_v:.3f} m")
    ax.axvline(median_v, color="black", linestyle="--", linewidth=1.2,
               label=f"median = {median_v:.3f} m")
    ax.axvline(rmse_v, color="black", linestyle=":", linewidth=1.2,
               label=f"RMSE = {rmse_v:.3f} m")
    ax.set_xlabel("ATE (m)")
    ax.set_ylabel("Count")
    ax.set_title(f"Robot {robot_char}: ATE Distribution")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right")
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_trajectories_topdown_compare(traj_gt,
                                      traj_odom,
                                      traj_results: list,
                                      labels: list[str],
                                      landmark_xyz: np.ndarray,
                                      hull,
                                      robot_char: str,
                                      save_path: str | None = None) -> None:
    """Top-down (XY) projection of GT, odometry, and N optimized trajectories.

    Companion to the 3D comparison view. Drops the depth axis so the
    horizontal beacon footprint and trajectory layout read directly. The
    color palette matches the 3D / APE compare plots so a line here
    corresponds to the same-color trace in the other views.

    Args:
        traj_gt, traj_odom, traj_results: evo PosePath3D instances.
        labels: One legend label per entry in traj_results.
        landmark_xyz: (N, 3) landmark positions; the Z column is dropped.
                      Pass an empty array to skip beacon markers.
        hull: LandmarkHull or None. When not None, the 2D hull polygon
              outline is drawn under the trajectories.
    """
    palette = ['tab:blue', 'tab:purple', 'tab:red', 'tab:brown',
               'tab:pink', 'tab:cyan', 'tab:olive', 'magenta']
    estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]

    fig, ax = plt.subplots(figsize=(8, 8))

    if hull is not None:
        loop = np.vstack([hull.xy_vertices, hull.xy_vertices[:1]])
        ax.fill(loop[:, 0], loop[:, 1],
                facecolor="tab:green", alpha=0.10,
                edgecolor="tab:green", linewidth=1.2,
                label="LBL hull (XY)", zorder=0)

    gt_xy = traj_gt.positions_xyz[:, :2]
    odom_xy = traj_odom.positions_xyz[:, :2]
    ax.plot(gt_xy[:, 0], gt_xy[:, 1], '--', color='black',
            linewidth=1.2, label='Ground Truth', zorder=2)
    ax.plot(odom_xy[:, 0], odom_xy[:, 1], '--', color='tab:orange',
            linewidth=1.0, alpha=0.8,
            label='Odometry Only Trajectory', zorder=2)
    for i, (traj, legend) in enumerate(zip(traj_results, estimate_legends)):
        xy = traj.positions_xyz[:, :2]
        ax.plot(xy[:, 0], xy[:, 1], '-',
                color=palette[i % len(palette)],
                linewidth=1.0, label=legend, zorder=3)

    if landmark_xyz.size:
        ax.scatter(landmark_xyz[:, 0], landmark_xyz[:, 1],
                   marker='*', s=120, color='gold',
                   edgecolor='black', linewidth=0.8,
                   label='Landmarks', zorder=4)

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"Robot {robot_char} — Top-Down Trajectory Comparison")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_range_depth_disparity_collaborative(fg,
                                             save_path: str | None = None) -> None:
    """Per-pose-index RSS of inter-endpoint depth disparities, collaborative.

    Aggregates every range edge in the factor graph (peer-to-peer and
    peer-to-landmark) into a single trace indexed by pose index. At each
    bin the value is `sqrt(sum(|Δz_i|^2))` over every range edge that
    happened at that pose index. Dips mark times when the collaborative
    range geometry is mostly horizontal, so paths sample little of the
    sound-speed profile and the refracted range residual collapses
    toward the straight-line value.

    All depths come from ground-truth `true_position[2]` so the metric
    reflects measurement geometry, not estimator state.

    Args:
        fg: PyFG factor graph (provides pose_variables, landmark_variables,
            range_measurements).
        save_path: If provided, save figure at this path.
    """
    z_lookup: dict[str, float] = {}
    idx_lookup: dict[str, int] = {}
    for chain in fg.pose_variables:
        for i, p in enumerate(chain):
            z_lookup[p.name] = float(p.true_position[2])
            idx_lookup[p.name] = i
    for lm in fg.landmark_variables:
        z_lookup[lm.name] = float(lm.true_position[2])

    n_poses = max((len(c) for c in fg.pose_variables), default=0)
    if n_poses == 0:
        return
    sumsq = np.zeros(n_poses)
    count = np.zeros(n_poses, dtype=int)

    for rm in fg.range_measurements:
        name_a, name_b = rm.association
        if name_a not in z_lookup or name_b not in z_lookup:
            continue
        a_is_pose = name_a in idx_lookup
        b_is_pose = name_b in idx_lookup
        if a_is_pose and b_is_pose:
            bin_idx = min(idx_lookup[name_a], idx_lookup[name_b])
        elif a_is_pose:
            bin_idx = idx_lookup[name_a]
        elif b_is_pose:
            bin_idx = idx_lookup[name_b]
        else:
            continue
        if bin_idx >= n_poses:
            continue
        sumsq[bin_idx] += (z_lookup[name_a] - z_lookup[name_b]) ** 2
        count[bin_idx] += 1

    populated = np.where(count > 0)[0]
    rss = np.sqrt(sumsq[populated])

    fig, ax = plt.subplots(figsize=(11, 3.5))
    ax.scatter(populated, rss,
               s=22, color='tab:blue',
               edgecolor='black', linewidth=0.4,
               alpha=0.85, zorder=3,
               label=r'$D_t$')
    ax.set_xlabel(r"Tick $t$")
    ax.set_ylabel(r"$D_t$ (m)")
    ax.set_title("Collaborative Range-Edge Depth Disparity")
    ax.set_xlim(0, n_poses - 1)
    ax.set_ylim(bottom=0)
    ax.minorticks_on()
    ax.grid(which='major', linestyle='-', linewidth=0.6, alpha=0.45, zorder=0)
    ax.grid(which='minor', linestyle=':', linewidth=0.4, alpha=0.25, zorder=0)
    ax.legend(loc="best", fontsize=9, framealpha=0.9)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_trajectory_xyz_stacked_compare(traj_gt,
                                        traj_odom,
                                        traj_results: list,
                                        labels: list[str],
                                        robot_char: str,
                                        save_path: str | None = None) -> None:
    """Per-axis stacked view of GT, odometry, and N optimized trajectories.

    Plots X(t), Y(t), Z(t) as three vertically stacked subplots against
    pose index. Unlike the top-down projection, near-vertical mission
    segments (descents and ascents) do not collapse into clusters here,
    so estimator divergence remains legible during cycling profiles.
    Color and dash conventions match the 3D and top-down comparison
    views.
    """
    palette = ['tab:blue', 'tab:purple', 'tab:red', 'tab:brown',
               'tab:pink', 'tab:cyan', 'tab:olive', 'magenta']
    estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]

    fig, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, sharex=True,
                                           figsize=(9, 8))

    def _plot_axes(traj, **kwargs):
        idx = np.arange(traj.positions_xyz.shape[0])
        ax_x.plot(idx, traj.positions_xyz[:, 0], **kwargs)
        ax_y.plot(idx, traj.positions_xyz[:, 1], **kwargs)
        ax_z.plot(idx, traj.positions_xyz[:, 2], **kwargs)

    _plot_axes(traj_gt, linestyle='--', color='black',
               linewidth=1.2, label='Ground Truth', zorder=2)
    _plot_axes(traj_odom, linestyle='--', color='tab:orange',
               linewidth=1.0, alpha=0.8,
               label='Odometry Only Trajectory', zorder=2)
    for i, (traj, legend) in enumerate(zip(traj_results, estimate_legends)):
        _plot_axes(traj, linestyle='-',
                   color=palette[i % len(palette)],
                   linewidth=1.0, label=legend, zorder=3)

    ax_x.set_ylabel("X (m)")
    ax_y.set_ylabel("Y (m)")
    ax_z.set_ylabel("Depth (m)")
    ax_z.invert_yaxis()
    ax_z.set_xlabel("Pose index")
    ax_x.set_title(f"Robot {robot_char} — Per-Axis Trajectory Comparison")
    for ax in (ax_x, ax_y, ax_z):
        ax.grid(True, alpha=0.3)
    ax_x.legend(loc="best", fontsize=9)
    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_ape_distribution_compare(ape_results: list[np.ndarray],
                                  labels: list[str],
                                  ape_odom: np.ndarray | None = None,
                                  robot_char: str = "",
                                  save_path: str | None = None,
                                  title: str | None = None) -> None:
    """evo-style APE distribution view: violin per series.

    Quartiles are drawn inline on each violin, preserving the box-plot
    information without the visual heaviness. Colors match the palette
    used in the line-plot view in `compare_results`.

    `ape_results` is a list of 1D APE arrays (one per series) and `ape_odom`
    is an optional odometry-only APE array; pass None to omit the odometry
    violin (e.g. when plotting pooled MC samples that have no per-pose odom
    reference).
    """
    if not ape_results:
        return
    if (any(len(a) < 2 for a in ape_results)
            or (ape_odom is not None and len(ape_odom) < 2)):
        print(f"[ape-dist] robot {robot_char}: too few poses, skipping plot")
        return

    palette = ['tab:blue', 'tab:purple', 'tab:red', 'tab:brown',
               'tab:pink', 'tab:cyan', 'tab:olive', 'magenta']
    estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]
    series_colors = [palette[i % len(palette)] for i in range(len(ape_results))]
    all_apes = list(ape_results)
    raw_labels = list(estimate_legends)
    if ape_odom is not None:
        all_apes.append(ape_odom)
        raw_labels.append("Odometry Only")
        series_colors.append("tab:orange")

    long_df = pd.DataFrame({
        "series": np.concatenate(
            [np.full(len(a), lbl) for a, lbl in zip(all_apes, raw_labels)]),
        "ATE (m)": np.concatenate(all_apes),
    })

    fig, ax_violin = plt.subplots(
        figsize=(max(9, 1.9 * len(all_apes)), 6))

    sns.violinplot(
        data=long_df, x="series", y="ATE (m)",
        hue="series", palette=dict(zip(raw_labels, series_colors)),
        inner="quartile", cut=0, ax=ax_violin, legend=False)
    ax_violin.set_xticks(range(len(raw_labels)))
    ax_violin.set_xticklabels(
        [textwrap.fill(t, width=18) for t in raw_labels])
    ax_violin.set_xlabel("")
    if title is None:
        if len(robot_char) == 1:
            title = f"Robot {robot_char}: ATE Distribution Comparison"
        elif robot_char:
            title = f"ATE Distribution Comparison ({robot_char})"
        else:
            title = "ATE Distribution Comparison"
    ax_violin.set_title(title)
    ax_violin.grid(axis="y", which="major", linestyle="-", color="#333333",
                   linewidth=0.9, alpha=0.55)
    ax_violin.grid(axis="y", which="minor", linestyle="-", color="#888888",
                   linewidth=0.4, alpha=0.25)
    ax_violin.set_axisbelow(True)

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_paired_ape_delta(delta: np.ndarray,
                          seeds: list[int] | np.ndarray,
                          save_dir: str | None = None,
                          prefix: str = "mc",
                          minuend_label: str = "Straight-line",
                          subtrahend_label: str = "Refracted",
                          robot_char: str = "",
                          show: bool = True) -> None:
    """Plot per-pose paired APE difference across seeds.

    `delta` is shape (n_seeds, n_poses), `delta[i, p] = APE(minuend)[p] -
    APE(subtrahend)[p]` under noise seed i. With the same seed used for both
    legs, odom/depth/range injection noise cancels and the per-pose
    difference isolates the mean-shift between the two range sources
    (e.g. refraction bias vs. straight-line mean).

    Renders, back to front:
      - per-seed traces (very faint gray, rasterized) as background texture
      - IQR (25-75%) band
      - min and max envelope (dashed)
      - mean curve (solid, bold)
      - y=0 reference line

    Note: matplotlib alpha-blends overlapping segments even within a single
    LineCollection; the very low alpha and rasterization keep overlap
    darkening mild rather than removing it entirely.
    """
    delta = np.asarray(delta, dtype=np.float64)
    if delta.ndim != 2:
        raise ValueError(
            f"delta must be 2D (n_seeds, n_poses), got shape {delta.shape}")

    n_seeds, n_poses = delta.shape
    x = np.arange(n_poses)

    mean_curve = np.mean(delta, axis=0)
    min_curve = np.min(delta, axis=0)
    max_curve = np.max(delta, axis=0)
    q25 = np.percentile(delta, 25, axis=0)
    q75 = np.percentile(delta, 75, axis=0)

    fig, ax = plt.subplots(figsize=(10, 5))

    segments = [np.column_stack([x, delta[i]]) for i in range(n_seeds)]
    seeds_lc = LineCollection(segments,
                              colors="lightsteelblue",
                              linewidths=0.8,
                              alpha=0.55,
                              zorder=0)
    seeds_lc.set_rasterized(True)
    ax.add_collection(seeds_lc)

    ax.fill_between(x, q25, q75, color="tab:blue", alpha=0.50,
                    label="IQR (25-75%)", zorder=1)
    ax.plot(x, min_curve, color="indianred", linewidth=1.0, linestyle="--",
            label="Min / Max", zorder=2)
    ax.plot(x, max_curve, color="indianred", linewidth=1.0, linestyle="--",
            zorder=2)
    ax.plot(x, mean_curve, color="black", linewidth=2.0,
            label="Mean", zorder=3)
    ax.axhline(0.0, color="black", linewidth=0.8, linestyle=":", zorder=4)
    ax.set_xlabel("Pose index")
    ax.set_ylabel(f"ATE({minuend_label}) - ATE({subtrahend_label})  [m]")
    if len(robot_char) == 1:
        title = (f"Robot {robot_char}: Paired per-pose ATE difference  "
                 f"(N={n_seeds} seeds)")
    elif robot_char:
        title = (f"Paired per-pose ATE difference ({robot_char})  "
                 f"(N={n_seeds} seeds)")
    else:
        title = f"Paired per-pose ATE difference  (N={n_seeds} seeds)"
    ax.set_title(title)
    handles, labels = ax.get_legend_handles_labels()
    legend_order = ["Mean", "Min / Max", "IQR (25-75%)"]
    ordered = [(handles[labels.index(name)], name)
               for name in legend_order if name in labels]
    if ordered:
        ax.legend([h for h, _ in ordered], [n for _, n in ordered],
                  loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_dir is not None:
        fig.savefig(os.path.join(save_dir, f"{prefix}_paired_ape_delta.png"),
                    dpi=300)
    if show:
        plt.show()


def plot_collaborative_ape_box_compare(per_robot_ape: list[tuple[str, list[metrics.APE]]],
                                       labels: list[str],
                                       save_path: str | None = None) -> None:
    """Per-robot paired box plot of estimator APE distributions.

    Each robot occupies an x-axis group containing one box per estimator label.
    Within each group, any "Straight-line*" label is placed first and the remaining
    labels follow their order in `labels`. Outliers are suppressed so the
    visual focus stays on median, IQR, and whiskers.
    """
    if len(per_robot_ape) < 2:
        return
    if any(len(ape.error) < 2 for _, apes in per_robot_ape for ape in apes):
        print("[collab-box] too few poses on at least one robot, skipping plot")
        return

    def _straight_line_first(lbls: list[str]) -> list[str]:
        straights = [l for l in lbls if "straight" in l.lower()]
        others = [l for l in lbls if "straight" not in l.lower()]
        return straights + others

    ordered_labels = _straight_line_first(labels)
    palette = {l: ("tab:purple" if "straight" in l.lower() else "tab:blue")
               for l in ordered_labels}

    rows = []
    for robot_char, ape_results in per_robot_ape:
        for label, ape in zip(labels, ape_results):
            rows.append(pd.DataFrame({
                "Robot": [f"Robot {robot_char}"] * len(ape.error),
                "Measurement": [label] * len(ape.error),
                "ATE (m)": ape.error,
            }))
    long_df = pd.concat(rows, ignore_index=True)
    long_df["Measurement"] = pd.Categorical(
        long_df["Measurement"], categories=ordered_labels, ordered=True)

    fig, ax = plt.subplots(figsize=(max(10, 2.4 * len(per_robot_ape)), 5))
    sns.boxplot(
        data=long_df, x="Robot", y="ATE (m)", hue="Measurement",
        hue_order=ordered_labels, palette=palette,
        width=0.6, showfliers=False, ax=ax)
    ax.set_xlabel("")
    ax.set_title("Collaborative ATE Distribution: Straight-line vs Refracted Ranges")
    ax.grid(True, alpha=0.3, axis="y")
    ax.legend(loc="upper right")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def plot_collaborative_ape_violin_aggregate(
        per_robot_ape: list[tuple[str, list[metrics.APE]]],
        labels: list[str],
        save_path: str | None = None) -> None:
    """Pooled-fleet violin plot: one violin per measurement.

    Concatenates each measurement's per-pose APE across all robots and shows
    the resulting distributions as side-by-side violins (any "Straight-line*" label
    first, then the remaining labels in `labels` order). Quartiles are drawn
    inline. Odometry is excluded so the comparison stays focused on the
    measurement conditions.
    """
    if len(per_robot_ape) < 2:
        return
    if any(len(ape.error) < 2 for _, apes in per_robot_ape for ape in apes):
        print("[collab-violin] too few poses on at least one robot, skipping plot")
        return

    ordered_labels = (
        [l for l in labels if "straight" in l.lower()]
        + [l for l in labels if "straight" not in l.lower()])
    palette = {l: ("tab:purple" if "straight" in l.lower() else "tab:blue")
               for l in ordered_labels}

    rows = []
    for _, ape_results in per_robot_ape:
        for label, ape in zip(labels, ape_results):
            rows.append(pd.DataFrame({
                "Measurement": [label] * len(ape.error),
                "ATE (m)": ape.error,
            }))
    long_df = pd.concat(rows, ignore_index=True)
    long_df["Measurement"] = pd.Categorical(
        long_df["Measurement"], categories=ordered_labels, ordered=True)

    fig, ax = plt.subplots(figsize=(7, 5))
    sns.violinplot(
        data=long_df, x="Measurement", y="ATE (m)",
        hue="Measurement", order=ordered_labels,
        palette=palette, inner="quartile", cut=0,
        ax=ax, legend=False)
    ax.set_xlabel("")
    ax.set_title(
        "Collaborative ATE Distribution Across Fleet: "
        "Straight-line vs Refracted Ranges")
    ax.grid(True, alpha=0.3, axis="y")

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=300)


def visualize(solver: FactorGraphSolver, save_dir: str | None = None,
              prefix: str = "", show_range_error: bool = True,
              estimate_label: str = "Estimate",
              show_landmark_hull: bool = False,
              show: bool = True):
    """Plot ground-truth, initial, and optimized trajectories with APE.

    Per robot, shows:
      - 3D trajectory overlay (ground-truth / initial / optimized),
        z-axis inverted (depth positive down), capped at -5m above surface
      - APE over pose index with range measurement locations marked
      - (optional) Range measurement error subplot below APE

    Args:
        solver: Solved FactorGraphSolver instance.
        save_dir: Directory to save figures. If None, only shows interactively.
        prefix: Filename prefix for saved figures (e.g. "measured", "true").
        show_range_error: Show range error subplot below APE. Disable for
                          true-range runs where error is always zero.
        show_landmark_hull: Tint the APE plot green where the GT pose lies
                            inside the 2D-XY convex hull of the LBL beacons.
                            Off by default.
    """
    if solver.result is None:
        raise RuntimeError("Call solver.solve() before visualize()")

    for robot_idx, pose_chain in enumerate(solver.fg.pose_variables):
        if not pose_chain:
            continue

        keys_ordered = [solver.key_map[p.name] for p in pose_chain]
        robot_char = pose_chain[0].name[0]

        traj_gt = extract_trajectory(solver.gt_values, keys_ordered)
        traj_odom = extract_trajectory(solver.odom_values, keys_ordered)
        traj_opt = extract_trajectory(solver.result, keys_ordered)

        ape_odom = metrics.APE(metrics.PoseRelation.translation_part)
        ape_odom.process_data((traj_gt, traj_odom))

        ape_opt = metrics.APE(metrics.PoseRelation.translation_part)
        ape_opt.process_data((traj_gt, traj_opt))

        estimate_legend = f"Estimated Traj w/ {estimate_label}"

        print(f"\nRobot {robot_char}: ATE (translation, {len(pose_chain)} poses):")
        print(f"  {'':20s} {'Odometry Only':>14s}  {estimate_legend:>32s}")
        for stat_name in ape_odom.get_all_statistics():
            val_d = ape_odom.get_all_statistics()[stat_name]
            val_o = ape_opt.get_all_statistics()[stat_name]
            print(f"  {stat_name:20s} {val_d:14.6f}  {val_o:32.6f}")

        fig = plt.figure(figsize=(12, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xyz)
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_gt,
                      style='--', color='black', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_odom,
                      style='--', color='tab:orange', label='Odometry Only Trajectory')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_opt,
                      style='-', color='tab:blue', label=estimate_legend)
        ax.invert_zaxis()
        ax.set_zlabel("Depth (m)")
        ax.legend()
        ax.set_title(f"Robot {robot_char} — Trajectory Comparison")

        range_indices, gps_indices = _measurement_pose_indices(solver, pose_chain)

        if show_range_error:
            fig2, (ax_ape, ax_range) = plt.subplots(
                2, 1, figsize=(10, 6), sharex=True,
                gridspec_kw={'height_ratios': [2, 1]})
        else:
            fig2 = plt.figure(figsize=(10, 4))
            ax_ape = fig2.add_subplot(111)

        if show_landmark_hull and solver.fg.landmark_variables:
            landmark_xyz = np.array(
                [np.asarray(lm.true_position, dtype=float)
                 for lm in solver.fg.landmark_variables],
                dtype=np.float64)
            hull = build_landmark_hull(landmark_xyz)
            inside_mask = inside_hull_mask(traj_gt.positions_xyz, hull)
            _shade_mask_spans(ax_ape, inside_mask,
                              color="tab:green", alpha=0.12,
                              label="Inside LBL hull")

        ax_ape.plot(ape_odom.error, color='tab:orange', linewidth=0.8,
                    alpha=0.6, label='Odometry Only Trajectory')
        ax_ape.plot(ape_opt.error, color='tab:blue', linewidth=0.8,
                    label=estimate_legend)
        if gps_indices:
            gps_list = sorted(gps_indices)
            ax_ape.scatter(gps_list, [ape_opt.error[i] for i in gps_list],
                           marker='x', s=12, color='orange', alpha=0.6,
                           zorder=3, label='GPS prior')
        if range_indices:
            rng_list = sorted(range_indices)
            ax_ape.scatter(rng_list, [ape_opt.error[i] for i in rng_list],
                           marker='*', s=16, color='purple', alpha=0.6,
                           zorder=3, label='Range measurement')
        ax_ape.set_ylabel("ATE (m)")
        ax_ape.set_title(f"Robot {robot_char}: Absolute Translation Error")
        ax_ape.legend()
        ax_ape.grid(True, alpha=0.3)

        if show_range_error:
            name_to_idx = {p.name: i for i, p in enumerate(pose_chain)}
            pose_names = set(name_to_idx.keys())
            true_fg = make_all_ranges_perfect(solver.fg)

            range_idx_list = []
            range_err_list = []
            for meas, true_meas in zip(solver.fg.range_measurements,
                                       true_fg.range_measurements):
                if true_meas.dist == 0:
                    continue
                error = meas.dist - true_meas.dist
                for name in meas.association:
                    if name in pose_names:
                        range_idx_list.append(name_to_idx[name])
                        range_err_list.append(error)

            if range_idx_list:
                ax_range.scatter(range_idx_list, range_err_list,
                                 s=8, alpha=0.5, color='purple')
                ax_range.axhline(y=0, color='black', linestyle='--',
                                 linewidth=0.5)
            ax_range.set_xlabel("Pose index")
            ax_range.set_ylabel("Range Error (m)")
            ax_range.grid(True, alpha=0.3)
        else:
            ax_ape.set_xlabel("Pose index")

        fig2.tight_layout()

        if save_dir:
            tag = f"{prefix}_" if prefix else ""
            fig.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_trajectory.png"), dpi=300)
            fig2.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_ape.png"), dpi=300)

        tag = f"{prefix}_" if prefix else ""
        hist_path = (os.path.join(save_dir, f"{tag}robot_{robot_char}_ape_dist.png")
                     if save_dir else None)
        plot_ape_histogram(ape_opt, ape_odom, robot_char,
                           estimate_label, save_path=hist_path)

    if show:
        plt.show()


def compare_results(solvers: list[FactorGraphSolver],
                    labels: list[str],
                    save_dir: str | None = None,
                    prefix: str = "",
                    show_landmark_hull: bool = False,
                    show_topdown: bool = False,
                    show_xyz_stack: bool = True,
                    show: bool = True):
    """Compare multiple solver results against a shared ground truth.

    Plots all optimized trajectories on the same 3D figure and overlays
    their APE curves for direct comparison. Ground truth and odometry
    are taken from solvers[0].

    For a fair comparison, all solvers should share the same fg input and
    perturbation config (odom_noise_sigmas + seed) so that odometry
    realizations are identical.

    Args:
        solvers: List of solved FactorGraphSolver instances.
        labels:  One label per solver (e.g. ["Bellhop Ranges", "True Ranges"]).
        save_dir: Directory to save figures. If None, only shows interactively.
        prefix:  Filename prefix for saved figures.
        show_landmark_hull: Tint the APE plot green where the GT pose lies
                            inside the 2D-XY convex hull of the LBL beacons,
                            and label entry/exit pose indices. Off by default.
        show_topdown: Save an additional top-down (XY) trajectory comparison
                      figure as `{prefix}robot_{char}_compare_trajectory_topdown.png`.
                      Off by default; only useful for static-depth missions
                      where descents and ascents do not collapse the projection.
        show_xyz_stack: Save a per-axis (X(t), Y(t), Z(t)) stacked trajectory
                        comparison figure as
                        `{prefix}robot_{char}_compare_trajectory_xyz.png`.
                        On by default.
    """
    if len(solvers) != len(labels):
        raise ValueError(f"Got {len(solvers)} solvers but {len(labels)} labels")
    for i, s in enumerate(solvers):
        if s.result is None:
            raise RuntimeError(f"Solver '{labels[i]}' has no result — call solve() first")

    ref = solvers[0]
    # Explicit palette for estimate trajectories.
    # Avoids black (GT), tab:orange (odometry), and greens; ensures no repeats.
    colors = ['tab:blue', 'tab:purple', 'tab:red', 'tab:brown',
              'tab:pink', 'tab:cyan', 'tab:olive', 'magenta']

    per_robot_ape: list[tuple[str, list[metrics.APE]]] = []

    for robot_idx, pose_chain in enumerate(ref.fg.pose_variables):
        if not pose_chain:
            continue

        keys_ordered = [ref.key_map[p.name] for p in pose_chain]
        robot_char = pose_chain[0].name[0]

        traj_gt = extract_trajectory(ref.gt_values, keys_ordered)
        traj_odom = extract_trajectory(ref.odom_values, keys_ordered)

        ape_odom = metrics.APE(metrics.PoseRelation.translation_part)
        ape_odom.process_data((traj_gt, traj_odom))

        # Compute APE for each solver
        ape_results = []
        traj_results = []
        for solver in solvers:
            traj = extract_trajectory(solver.result, keys_ordered)
            traj_results.append(traj)
            ape = metrics.APE(metrics.PoseRelation.translation_part)
            ape.process_data((traj_gt, traj))
            ape_results.append(ape)
        per_robot_ape.append((robot_char, ape_results))

        # Print comparison table
        estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]
        col_w = max(14, *(len(l) + 2 for l in estimate_legends))
        print(f"\nRobot {robot_char}: ATE (translation, {len(pose_chain)} poses):")
        header = f"  {'':20s} {'Odometry Only':>{col_w}s}"
        for legend in estimate_legends:
            header += f"  {legend:>{col_w}s}"
        print(header)

        for stat_name in ape_odom.get_all_statistics():
            row = f"  {stat_name:20s} {ape_odom.get_all_statistics()[stat_name]:{col_w}.6f}"
            for ape in ape_results:
                row += f"  {ape.get_all_statistics()[stat_name]:{col_w}.6f}"
            print(row)

        # Figure 1: 3D trajectory comparison
        fig = plt.figure(figsize=(12, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xyz)
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_gt,
                      style='--', color='black', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_odom,
                      style='--', color='tab:orange', label='Odometry Only Trajectory')
        for i, (traj, legend) in enumerate(zip(traj_results, estimate_legends)):
            evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj,
                          style='-', color=colors[i % len(colors)], label=legend)
        ax.invert_zaxis()
        ax.set_zlabel("Depth (m)")
        ax.legend()
        ax.set_title(f"Robot {robot_char} — Trajectory Comparison")

        # Shared landmark/hull data for both APE shading and top-down view.
        if ref.fg.landmark_variables:
            landmark_xyz = np.array(
                [np.asarray(lm.true_position, dtype=float)
                 for lm in ref.fg.landmark_variables],
                dtype=np.float64)
        else:
            landmark_xyz = np.empty((0, 3), dtype=np.float64)
        hull = (build_landmark_hull(landmark_xyz)
                if landmark_xyz.size else None)

        # Figure 2: APE comparison
        fig2 = plt.figure(figsize=(10, 4))
        ax2 = fig2.add_subplot(111)

        inside_mask = np.zeros(0, dtype=bool)
        if show_landmark_hull and hull is not None:
            inside_mask = inside_hull_mask(traj_gt.positions_xyz, hull)
            _shade_mask_spans(ax2, inside_mask,
                              color="tab:green", alpha=0.12,
                              label="Inside LBL hull")

        ax2.plot(ape_odom.error, color='tab:orange', linewidth=0.8,
                 alpha=0.6, linestyle='--', label='Odometry Only Trajectory')
        for i, (ape, legend) in enumerate(zip(ape_results, estimate_legends)):
            ax2.plot(ape.error, color=colors[i % len(colors)],
                     linewidth=0.8, label=legend)
        ax2.set_xlabel("Pose index")
        ax2.set_ylabel("ATE (m)")
        ax2.set_title(f"Robot {robot_char}: Absolute Translation Error Comparison")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        if show_landmark_hull:
            _label_mask_boundaries(ax2, inside_mask)
            if inside_mask.any():
                entries, exits = _mask_run_boundaries(inside_mask)
                spans = list(zip(entries.tolist(), exits.tolist()))
                print(f"Robot {robot_char} — hull spans (pose idx): {spans}")

        fig2.tight_layout()

        tag = f"{prefix}_" if prefix else ""

        if save_dir:
            fig.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_compare_trajectory.png"), dpi=300)
            # Full autoscale version preserves the odometry baseline tail.
            fig2.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_compare_ape_full.png"), dpi=300)

        # Clip y-axis to the range-aided traces so the odometry baseline does
        # not dominate the scale and squash range-aided detail. The odometry
        # line still enters from below and clips off the top of the axes.
        if ape_results:
            y_top = 1.15 * max(float(np.max(ape.error)) for ape in ape_results)
            ax2.set_ylim(0, y_top)

        if save_dir:
            fig2.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_compare_ape.png"), dpi=300)

        dist_path = (os.path.join(save_dir,
                                  f"{tag}robot_{robot_char}_compare_ape_dist.png")
                     if save_dir else None)
        plot_ape_distribution_compare(
            [a.error for a in ape_results], labels,
            ape_odom=ape_odom.error,
            robot_char=robot_char, save_path=dist_path)

        if show_topdown:
            topdown_path = (os.path.join(
                save_dir,
                f"{tag}robot_{robot_char}_compare_trajectory_topdown.png")
                if save_dir else None)
            plot_trajectories_topdown_compare(
                traj_gt, traj_odom, traj_results, labels,
                landmark_xyz, hull, robot_char, save_path=topdown_path)

        if show_xyz_stack:
            xyz_path = (os.path.join(
                save_dir,
                f"{tag}robot_{robot_char}_compare_trajectory_xyz.png")
                if save_dir else None)
            plot_trajectory_xyz_stacked_compare(
                traj_gt, traj_odom, traj_results, labels,
                robot_char, save_path=xyz_path)

    if save_dir:
        tag = f"{prefix}_" if prefix else ""
        plot_range_depth_disparity_collaborative(
            ref.fg,
            save_path=os.path.join(save_dir,
                                   f"{tag}range_depth_disparity.png"))
        if len(per_robot_ape) >= 2:
            plot_collaborative_ape_box_compare(
                per_robot_ape, labels,
                save_path=os.path.join(save_dir,
                                       f"{tag}collaborative_compare_ape_box.png"))
            plot_collaborative_ape_violin_aggregate(
                per_robot_ape, labels,
                save_path=os.path.join(save_dir,
                                       f"{tag}collaborative_compare_ape_violin.png"))

    if show:
        plt.show()


def compare_depth_error(solvers: list[FactorGraphSolver],
                        labels: list[str],
                        save_dir: str | None = None,
                        prefix: str = "",
                        show: bool = True):
    """Single-figure comparison of world-frame z error per solver.

    Concatenates poses across all robots into one x-axis so there is one line
    per solver. Vertical dotted lines mark robot boundaries when there is
    more than one robot.

    Args:
        solvers: List of solved FactorGraphSolver instances.
        labels:  One label per solver (e.g. ["Refracted Ranges", ...]).
        save_dir: Directory to save the figure.
        prefix:  Filename prefix.
    """
    if len(solvers) != len(labels):
        raise ValueError(f"Got {len(solvers)} solvers but {len(labels)} labels")
    for s, l in zip(solvers, labels):
        if s.result is None:
            raise RuntimeError(f"Solver '{l}' has no result — call solve() first")

    ref = solvers[0]
    estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]
    colors = ['tab:blue', 'tab:purple', 'tab:red', 'tab:brown',
              'tab:pink', 'tab:cyan', 'tab:olive', 'magenta']

    # Build concatenated GT z and odom z (from the first solver).
    gt_z_parts: list[np.ndarray] = []
    odom_z_parts: list[np.ndarray] = []
    boundaries: list[int] = []
    for pose_chain in ref.fg.pose_variables:
        if not pose_chain:
            continue
        keys = [ref.key_map[p.name] for p in pose_chain]
        gt_z_parts.append(np.array([p.true_position[2] for p in pose_chain]))
        odom_z_parts.append(
            np.array([ref.odom_values.atPose3(k).translation()[2] for k in keys]))
        boundaries.append(sum(len(p) for p in gt_z_parts))

    gt_z = np.concatenate(gt_z_parts)
    odom_z = np.concatenate(odom_z_parts)

    fig = plt.figure(figsize=(11, 4))
    ax = fig.add_subplot(111)
    ax.axhline(0, color='black', linestyle='--', linewidth=0.5)

    # Robot boundaries (only if multi-robot).
    if len(boundaries) > 1:
        for b in boundaries[:-1]:
            ax.axvline(b, color='gray', linestyle=':', linewidth=0.6, alpha=0.6)

    ax.plot(odom_z - gt_z, color='tab:orange', linestyle='--', alpha=0.6,
            linewidth=0.8, label='Odometry Only Trajectory')

    for i, (solver, legend) in enumerate(zip(solvers, estimate_legends)):
        opt_parts: list[np.ndarray] = []
        for pose_chain in ref.fg.pose_variables:
            if not pose_chain:
                continue
            keys = [solver.key_map[p.name] for p in pose_chain]
            opt_parts.append(
                np.array([solver.result.atPose3(k).translation()[2] for k in keys]))
        opt_z = np.concatenate(opt_parts)
        ax.plot(opt_z - gt_z, color=colors[i % len(colors)],
                linewidth=0.8, label=legend)

    ax.set_xlabel('Pose index (concatenated across robots)')
    ax.set_ylabel('Global z error (m)   [estimate − GT]')
    ax.set_title('Depth Error Comparison')
    ax.legend()
    ax.grid(True, alpha=0.3)
    fig.tight_layout()

    if save_dir:
        tag = f"{prefix}_" if prefix else ""
        fig.savefig(os.path.join(save_dir, f"{tag}compare_z_error.png"), dpi=300)

    if show:
        plt.show()


def visualize_landmarks(solver: FactorGraphSolver,
                        save_dir: str | None = None,
                        prefix: str = ""):
    """Plot ground truth vs estimated landmark positions and print error table.

    Produces a 3D scatter with GT (green) and estimated (blue) landmarks
    connected by dashed error lines, with dx/dy/dz annotations. Saves
    figure to save_dir without showing interactively.

    Args:
        solver: Solved FactorGraphSolver instance.
        save_dir: Directory to save figure. Required for output.
        prefix: Filename prefix for saved figure.
    """
    if solver.result is None:
        raise RuntimeError("Call solver.solve() before visualize_landmarks()")

    landmarks = solver.fg.landmark_variables
    if not landmarks:
        print("No landmarks to visualize.")
        return

    import numpy as np

    gt_pts = []
    est_pts = []
    names = []
    for lm in landmarks:
        key = solver.key_map[lm.name]
        gt = solver.gt_values.atPoint3(key)
        est = solver.result.atPoint3(key)
        gt_pts.append(gt)
        est_pts.append(est)
        names.append(lm.name)

    gt_pts = np.array(gt_pts)
    est_pts = np.array(est_pts)
    deltas = est_pts - gt_pts
    norms = np.linalg.norm(deltas, axis=1)

    # Print table
    print(f"\nLandmark Position Error ({prefix or 'default'}):")
    print(f"  {'Name':>6s}  {'GT_X':>10s}  {'GT_Y':>10s}  {'GT_Z':>10s}  "
          f"{'Est_X':>10s}  {'Est_Y':>10s}  {'Est_Z':>10s}  "
          f"{'dX':>10s}  {'dY':>10s}  {'dZ':>10s}  {'||err||':>10s}")
    print(f"  {'─' * 6}  {'─' * 10}  {'─' * 10}  {'─' * 10}  "
          f"{'─' * 10}  {'─' * 10}  {'─' * 10}  "
          f"{'─' * 10}  {'─' * 10}  {'─' * 10}  {'─' * 10}")
    for i, name in enumerate(names):
        g, e, d = gt_pts[i], est_pts[i], deltas[i]
        print(f"  {name:>6s}  {g[0]:10.2f}  {g[1]:10.2f}  {g[2]:10.2f}  "
              f"{e[0]:10.2f}  {e[1]:10.2f}  {e[2]:10.2f}  "
              f"{d[0]:+10.4f}  {d[1]:+10.4f}  {d[2]:+10.4f}  {norms[i]:10.4f}")

    # 3D scatter
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    ax.scatter(gt_pts[:, 0], gt_pts[:, 1], gt_pts[:, 2],
               c='green', s=80, marker='o', label='Ground Truth')
    ax.scatter(est_pts[:, 0], est_pts[:, 1], est_pts[:, 2],
               c='blue', s=80, marker='^', label='Estimated')

    for i, name in enumerate(names):
        g, e, d = gt_pts[i], est_pts[i], deltas[i]
        ax.plot([g[0], e[0]], [g[1], e[1]], [g[2], e[2]],
                'r--', linewidth=1, alpha=0.7)
        ax.text(g[0], g[1], g[2], f" {name}", fontsize=8, color='green')
        ax.text(e[0], e[1], e[2],
                f" d=[{d[0]:+.1f}, {d[1]:+.1f}, {d[2]:+.1f}]",
                fontsize=7, color='red')

    ax.invert_zaxis()
    ax.set_zlabel("Depth (m)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_title("Landmark Ground Truth vs Estimated")

    if save_dir:
        tag = f"{prefix}_" if prefix else ""
        fig.savefig(os.path.join(save_dir, f"{tag}landmarks.png"), dpi=300)
    plt.close(fig)
