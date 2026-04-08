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
              prefix: str = "", show_range_error: bool = True):
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
        traj_init = extract_trajectory(solver.initial, keys_ordered)
        traj_odom = extract_trajectory(solver.odom_values, keys_ordered)
        traj_opt = extract_trajectory(solver.result, keys_ordered)

        ape_init = metrics.APE(metrics.PoseRelation.translation_part)
        ape_init.process_data((traj_gt, traj_init))

        ape_odom = metrics.APE(metrics.PoseRelation.translation_part)
        ape_odom.process_data((traj_gt, traj_odom))

        ape_opt = metrics.APE(metrics.PoseRelation.translation_part)
        ape_opt.process_data((traj_gt, traj_opt))

        print(f"\nRobot {robot_char} — APE (translation, {len(pose_chain)} poses):")
        print(f"  {'':20s} {'Initial':>12s}  {'Odometry':>12s}  {'Optimized':>12s}")
        for stat_name in ape_init.get_all_statistics():
            val_i = ape_init.get_all_statistics()[stat_name]
            val_d = ape_odom.get_all_statistics()[stat_name]
            val_o = ape_opt.get_all_statistics()[stat_name]
            print(f"  {stat_name:20s} {val_i:12.6f}  {val_d:12.6f}  {val_o:12.6f}")

        fig = plt.figure(figsize=(12, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xyz)
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_gt,
                      style='-', color='green', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_init,
                      style='--', color='red', label='Initial')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_odom,
                      style='--', color='orange', label='Odometry (perturbed)')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_opt,
                      style='-', color='blue', label='Optimized')
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

        ax_ape.plot(ape_init.error, color='red', linewidth=0.8,
                    alpha=0.6, label='Initial')
        ax_ape.plot(ape_odom.error, color='orange', linewidth=0.8,
                    alpha=0.6, label='Odometry (perturbed)')
        ax_ape.plot(ape_opt.error, color='blue', linewidth=0.8,
                    label='Optimized')
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
        col_w = max(14, *(len(l) + 2 for l in labels))
        print(f"\nRobot {robot_char} — APE (translation, {len(pose_chain)} poses):")
        header = f"  {'':20s} {'Odometry':>{col_w}s}"
        for label in labels:
            header += f"  {label:>{col_w}s}"
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
                      style='--', color='orange', label='Odometry')
        for i, (traj, label) in enumerate(zip(traj_results, labels)):
            evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj,
                          style='-', color=colors[i % len(colors)], label=label)
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
                 alpha=0.6, linestyle='--', label='Odometry')
        for i, (ape, label) in enumerate(zip(ape_results, labels)):
            ax2.plot(ape.error, color=colors[i % len(colors)],
                     linewidth=0.8, label=label)
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
