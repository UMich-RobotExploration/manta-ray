#!/usr/bin/env python3
"""Visualization for FactorGraphSolver results.

Plots ground-truth, initial estimate, and optimized trajectories with APE
comparison using the evo library.
"""

from __future__ import annotations

import os

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from evo.core import metrics
from evo.tools import plot as evo_plot

from py_factor_graph.modifiers import make_all_ranges_perfect

from pyfg_to_gtsam import FactorGraphSolver, extract_trajectory


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


def visualize(solver: FactorGraphSolver, save_dir: str | None = None,
              prefix: str = "", show_range_error: bool = True,
              estimate_label: str = "Estimate"):
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

        print(f"\nRobot {robot_char} — APE (translation, {len(pose_chain)} poses):")
        print(f"  {'':20s} {'Odometry Only':>14s}  {estimate_legend:>32s}")
        for stat_name in ape_odom.get_all_statistics():
            val_d = ape_odom.get_all_statistics()[stat_name]
            val_o = ape_opt.get_all_statistics()[stat_name]
            print(f"  {stat_name:20s} {val_d:14.6f}  {val_o:32.6f}")

        fig = plt.figure(figsize=(12, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xyz)
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_gt,
                      style='-', color='green', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_odom,
                      style='--', color='orange', label='Odometry Only Trajectory')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_opt,
                      style='-', color='blue', label=estimate_legend)
        ax.invert_zaxis()
        ax.set_zlabel("Depth (m)")
        z_min, z_max = ax.get_zlim()
        ax.set_zlim(z_min, max(z_max, -5))
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

        ax_ape.plot(ape_odom.error, color='orange', linewidth=0.8,
                    alpha=0.6, label='Odometry Only Trajectory')
        ax_ape.plot(ape_opt.error, color='blue', linewidth=0.8,
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
        ax_ape.set_ylabel("APE (m)")
        ax_ape.set_title(f"Robot {robot_char} — Absolute Pose Error")
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
            fig.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_trajectory.png"), dpi=150)
            fig2.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_ape.png"), dpi=150)

    plt.show()


def compare_results(solvers: list[FactorGraphSolver],
                    labels: list[str],
                    save_dir: str | None = None,
                    prefix: str = ""):
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
    """
    if len(solvers) != len(labels):
        raise ValueError(f"Got {len(solvers)} solvers but {len(labels)} labels")
    for i, s in enumerate(solvers):
        if s.result is None:
            raise RuntimeError(f"Solver '{labels[i]}' has no result — call solve() first")

    ref = solvers[0]
    colors = plt.cm.tab10.colors

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

        # Print comparison table
        estimate_legends = [f"Estimated Traj w/ {l}" for l in labels]
        col_w = max(14, *(len(l) + 2 for l in estimate_legends))
        print(f"\nRobot {robot_char} — APE (translation, {len(pose_chain)} poses):")
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
                      style='-', color='green', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_odom,
                      style='--', color='orange', label='Odometry Only Trajectory')
        for i, (traj, legend) in enumerate(zip(traj_results, estimate_legends)):
            evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj,
                          style='-', color=colors[i % len(colors)], label=legend)
        ax.invert_zaxis()
        ax.set_zlabel("Depth (m)")
        z_min, z_max = ax.get_zlim()
        ax.set_zlim(z_min, max(z_max, -5))
        ax.legend()
        ax.set_title(f"Robot {robot_char} — Trajectory Comparison")

        # Figure 2: APE comparison
        fig2 = plt.figure(figsize=(10, 4))
        ax2 = fig2.add_subplot(111)
        ax2.plot(ape_odom.error, color='orange', linewidth=0.8,
                 alpha=0.6, linestyle='--', label='Odometry Only Trajectory')
        for i, (ape, legend) in enumerate(zip(ape_results, estimate_legends)):
            ax2.plot(ape.error, color=colors[i % len(colors)],
                     linewidth=0.8, label=legend)
        ax2.set_xlabel("Pose index")
        ax2.set_ylabel("APE (m)")
        ax2.set_title(f"Robot {robot_char} — Absolute Pose Error Comparison")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()

        if save_dir:
            tag = f"{prefix}_" if prefix else ""
            fig.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_compare_trajectory.png"), dpi=150)
            fig2.savefig(os.path.join(save_dir, f"{tag}robot_{robot_char}_compare_ape.png"), dpi=150)

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
    z_min, z_max = ax.get_zlim()
    ax.set_zlim(z_min, max(z_max, -5))
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.set_title("Landmark Ground Truth vs Estimated")

    if save_dir:
        tag = f"{prefix}_" if prefix else ""
        fig.savefig(os.path.join(save_dir, f"{tag}landmarks.png"), dpi=150)
    plt.close(fig)
