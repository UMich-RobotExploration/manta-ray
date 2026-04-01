#!/usr/bin/env python3
"""Visualization for FactorGraphSolver results.

Plots ground-truth, initial estimate, and optimized trajectories with APE
comparison using the evo library.
"""

from __future__ import annotations

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from evo.core import metrics
from evo.tools import plot as evo_plot

from pyfg_to_gtsam import FactorGraphSolver, extract_trajectory


def visualize(solver: FactorGraphSolver):
    """Plot ground-truth, initial, and optimized trajectories with APE.

    Per robot, shows:
      - 3D trajectory overlay (ground-truth / initial / optimized)
      - APE over pose index for both initial and optimized
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
        traj_opt = extract_trajectory(solver.result, keys_ordered)

        ape_init = metrics.APE(metrics.PoseRelation.translation_part)
        ape_init.process_data((traj_gt, traj_init))

        ape_opt = metrics.APE(metrics.PoseRelation.translation_part)
        ape_opt.process_data((traj_gt, traj_opt))

        print(f"\nRobot {robot_char} — APE (translation, {len(pose_chain)} poses):")
        print(f"  {'':20s} {'Initial':>12s}  {'Optimized':>12s}")
        for stat_name in ape_init.get_all_statistics():
            val_i = ape_init.get_all_statistics()[stat_name]
            val_o = ape_opt.get_all_statistics()[stat_name]
            print(f"  {stat_name:20s} {val_i:12.6f}  {val_o:12.6f}")

        fig = plt.figure(figsize=(12, 8))
        ax = evo_plot.prepare_axis(fig, evo_plot.PlotMode.xyz)
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_gt,
                      style='-', color='green', label='Ground Truth')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_init,
                      style='--', color='red', label='Initial')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_opt,
                      style='-', color='blue', label='Optimized')
        ax.legend()
        ax.set_title(f"Robot {robot_char} — Trajectory Comparison")

        fig2 = plt.figure(figsize=(10, 4))
        ax2 = fig2.add_subplot(111)
        ax2.plot(ape_init.error, color='red', linewidth=0.8,
                 alpha=0.6, label='Initial')
        ax2.plot(ape_opt.error, color='blue', linewidth=0.8,
                 label='Optimized')
        ax2.set_xlabel("Pose index")
        ax2.set_ylabel("APE (m)")
        ax2.set_title(f"Robot {robot_char} — Absolute Pose Error")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()

    plt.show()
