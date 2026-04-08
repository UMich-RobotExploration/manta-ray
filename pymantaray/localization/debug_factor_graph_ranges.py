#!/usr/bin/env python3
"""
Script that helps visualize and debug range data.
"""

import os

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Import parsing function
from py_factor_graph.io.pyfg_text import read_from_pyfg_text
from py_factor_graph.utils.plot_utils import (
    draw_pose_3d,
    draw_landmark_variable_3d,
    draw_traj_3d,
    draw_range_measurement_3d,
)
from py_factor_graph.variables import PoseVariable3D
from py_factor_graph.modifiers import make_all_ranges_perfect


FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-no-multi/output.pfg"
# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl/output.pfg"
WORK_DIR = os.path.dirname(FILE_PATH)


def plot_range_histogram(fg_data, save_dir: str | None = None):
    """Plot a histogram comparing measured range distances vs true distances.

    Args:
        fg_data: FactorGraphData object with range measurements
        save_dir: Directory to save figure. If None, only shows interactively.
    """
    if not fg_data.range_measurements:
        print("No range measurements to compare.")
        return

    true_fg = make_all_ranges_perfect(fg_data)

    measured_dists = [m.dist for m in fg_data.range_measurements]
    true_dists = [m.dist for m in true_fg.range_measurements]
    errors = [m - t for m, t in zip(measured_dists, true_dists)]
    pct_errors = [100.0 * (m - t) / t for m, t in zip(measured_dists, true_dists) if t != 0]

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))

    # Overlaid histograms of measured vs true
    ax1.hist(true_dists, bins=30, alpha=0.6, label='True', color='green')
    ax1.hist(measured_dists, bins=30, alpha=0.6, label='Measured', color='blue')
    ax1.set_xlabel('Distance')
    ax1.set_ylabel('Count')
    ax1.set_title('Measured vs True Range Distances')
    ax1.legend()

    # Error distribution
    ax2.hist(errors, bins=30, alpha=0.7, color='red')
    ax2.axvline(x=0, color='black', linestyle='--', linewidth=1)
    ax2.set_xlabel('Error (Measured - True)')
    ax2.set_ylabel('Count')
    ax2.set_title('Range Measurement Error Distribution')

    # Percent error distribution
    ax3.hist(pct_errors, bins=30, alpha=0.7, color='orange')
    ax3.axvline(x=0, color='black', linestyle='--', linewidth=1)
    ax3.set_xlabel('Percent Error (%)')
    ax3.set_ylabel('Count')
    ax3.set_title('Range Measurement Percent Error')

    plt.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_histogram.png"), dpi=150)
    plt.show()


def plot_range_diagnostics(fg_data, save_dir: str | None = None):
    """Plot range measurement error diagnostics broken down by association type.

    Categorizes range measurements into pose↔pose (robot-robot) and
    pose↔landmark (robot-landmark), then plots absolute error, percent error,
    and error vs measurement index for each category.

    Args:
        fg_data: FactorGraphData object with range measurements
        save_dir: Directory to save figure. If None, only shows interactively.
    """
    if not fg_data.range_measurements:
        print("No range measurements to diagnose.")
        return

    true_fg = make_all_ranges_perfect(fg_data)
    pose_keys = set(fg_data.pose_variables_dict.keys())

    categories = {
        "Robot ↔ Robot": {"measured": [], "true": []},
        "Robot ↔ Landmark": {"measured": [], "true": []},
    }

    for meas, true_meas in zip(fg_data.range_measurements, true_fg.range_measurements):
        name_a, name_b = meas.association
        a_is_pose = name_a in pose_keys
        b_is_pose = name_b in pose_keys

        if a_is_pose and b_is_pose:
            cat = "Robot ↔ Robot"
        elif a_is_pose or b_is_pose:
            cat = "Robot ↔ Landmark"
        else:
            continue

        categories[cat]["measured"].append(meas.dist)
        categories[cat]["true"].append(true_meas.dist)

    active_cats = {k: v for k, v in categories.items()
                   if len(v["measured"]) > 0}

    if not active_cats:
        print("No classifiable range measurements found.")
        return

    nrows = len(active_cats)
    fig, axes = plt.subplots(nrows, 3, figsize=(18, 5 * nrows))
    if nrows == 1:
        axes = axes[np.newaxis, :]

    for row, (cat_name, data) in enumerate(active_cats.items()):
        measured = np.array(data["measured"])
        true = np.array(data["true"])
        abs_err = measured - true
        pct_err = 100.0 * abs_err / true

        # Print summary
        print(f"\n{'=' * 50}")
        print(f"  {cat_name}  ({len(measured)} measurements)")
        print(f"{'=' * 50}")
        print(f"  Absolute error (m):  mean={np.mean(abs_err):.4f}  "
              f"std={np.std(abs_err):.4f}  max={np.max(np.abs(abs_err)):.4f}")
        print(f"  Percent error  (%):  mean={np.mean(pct_err):.2f}  "
              f"std={np.std(pct_err):.2f}  max={np.max(np.abs(pct_err)):.2f}")

        # Absolute error histogram
        ax = axes[row, 0]
        ax.hist(abs_err, bins=30, alpha=0.7, color='red')
        ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Absolute Error (m)")
        ax.set_ylabel("Count")
        ax.set_title(f"{cat_name} — Absolute Error")

        # Percent error histogram
        ax = axes[row, 1]
        ax.hist(pct_err, bins=30, alpha=0.7, color='orange')
        ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Percent Error (%)")
        ax.set_ylabel("Count")
        ax.set_title(f"{cat_name} — Percent Error")

        # Absolute error vs index
        ax = axes[row, 2]
        ax.scatter(range(len(abs_err)), abs_err, s=4, alpha=0.5, color='blue')
        ax.axhline(y=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Measurement Index")
        ax.set_ylabel("Absolute Error (m)")
        ax.set_title(f"{cat_name} — Error vs Index")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_diagnostics.png"), dpi=150)
    plt.show()


def print_worst_ranges(fg_data, n: int = 20):
    """Print a table of range measurements with the largest percent error.

    Columns: timestamp, source, receiver, measured, true, abs_err, pct_err
    Sorted by descending absolute percent error.
    """
    if not fg_data.range_measurements:
        print("No range measurements.")
        return

    true_fg = make_all_ranges_perfect(fg_data)

    rows = []
    for meas, true_meas in zip(fg_data.range_measurements, true_fg.range_measurements):
        if true_meas.dist == 0:
            continue
        abs_err = meas.dist - true_meas.dist
        pct_err = 100.0 * abs_err / true_meas.dist
        rows.append((
            meas.timestamp if meas.timestamp is not None else float('nan'),
            meas.association[0],
            meas.association[1],
            meas.dist,
            true_meas.dist,
            abs_err,
            pct_err,
        ))

    rows.sort(key=lambda r: abs(r[6]), reverse=True)

    print(f"\nTop {min(n, len(rows))} range measurements by percent error:")
    print(f"  {'Time':>10s}  {'Source':>8s}  {'Receiver':>8s}  "
          f"{'Measured':>10s}  {'True':>10s}  {'AbsErr':>10s}  {'PctErr':>8s}")
    print(f"  {'─' * 10}  {'─' * 8}  {'─' * 8}  "
          f"{'─' * 10}  {'─' * 10}  {'─' * 10}  {'─' * 8}")
    for row in rows[:n]:
        ts, src, rcv, measured, true, abs_e, pct_e = row
        print(f"  {ts:10.3f}  {src:>8s}  {rcv:>8s}  "
              f"{measured:10.4f}  {true:10.4f}  {abs_e:+10.4f}  {pct_e:+7.2f}%")


def plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True,
                         save_dir: str | None = None):
    """Plot 3D factor graph data.

    Args:
        fg_data: FactorGraphData object (must be 3D)
        show_trajectories: Whether to plot pose trajectories
        show_landmarks: Whether to plot landmark variables
        save_dir: Directory to save figure. If None, only shows interactively.
    """

    if fg_data.dimension != 3:
        raise ValueError(f"Data is {fg_data.dimension}D, not 3D")

    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Invert z-axis so positive z points downward (depth)
    ax.invert_zaxis()

    # Plot landmarks
    if show_landmarks and fg_data.landmark_variables:
        print(f"Plotting {len(fg_data.landmark_variables)} landmarks...")
        for landmark in fg_data.landmark_variables:
            draw_landmark_variable_3d(ax, landmark)

    # Plot range measurements
    if fg_data.range_measurements:
        print(f"Plotting {len(fg_data.range_measurements)} range measurements...")
        all_variables = fg_data.pose_and_landmark_variables_dict
        pose_variables = fg_data.pose_variables_dict
        skipped_ranges = 0

        for range_meas in fg_data.range_measurements:
            key_1, key_2 = range_meas.association
            var_1 = all_variables.get(key_1)
            var_2 = all_variables.get(key_2)

            # Skip malformed associations that do not resolve to known variables.
            if var_1 is None or var_2 is None:
                skipped_ranges += 1
                continue

            # draw_range_measurement_3d needs one endpoint to be a pose.
            if key_1 in pose_variables:
                from_pose = var_1
                to_variable = var_2
            elif key_2 in pose_variables:
                from_pose = var_2
                to_variable = var_1
            else:
                skipped_ranges += 1
                continue

            if not isinstance(from_pose, PoseVariable3D):
                skipped_ranges += 1
                continue

            draw_range_measurement_3d(
                ax,
                range_meas,
                from_pose,
                to_variable,
                add_line=True,
            )

        if skipped_ranges:
            print(f"Skipped {skipped_ranges} range measurements (no pose endpoint or unresolved key).")

    # Plot pose variables and trajectories
    print(f"Plotting {fg_data.num_robots} robot trajectories...")
    colors = ['blue', 'red', 'green', 'orange', 'purple']

    for robot_idx in range(fg_data.num_robots):
        color = colors[robot_idx % len(colors)]
        poses = fg_data.pose_variables[robot_idx]

        if show_trajectories and len(poses) > 0:
            # Extract x, y, z coordinates from poses
            x_traj = [pose.true_x for pose in poses]
            y_traj = [pose.true_y for pose in poses]
            z_traj = [pose.true_z for pose in poses]

            # Draw trajectory line
            draw_traj_3d(ax, x_traj, y_traj, z_traj, color=color)

            # Draw some poses along the trajectory (every Nth pose to avoid clutter)
            step = max(1, len(poses) // 10)
            for pose in poses[::step]:
                draw_pose_3d(ax, pose, color=color, scale=0.5)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Factor Graph - {fg_data.num_robots} Robot(s), {len(fg_data.landmark_variables)} Landmarks')

    if save_dir:
        fig.savefig(os.path.join(save_dir, "factor_graph_3d.png"), dpi=150)
    plt.show()


if __name__ == "__main__":
    # Read the factor graph
    print(f"Reading .pyfg file: {FILE_PATH}")
    fg_data = read_from_pyfg_text(FILE_PATH)

    # Print some info
    print(f"\nFactor Graph Info:")
    print(f"  Dimension: {fg_data.dimension}D")
    print(f"  Robots: {fg_data.num_robots}")
    print(f"  Landmarks: {len(fg_data.landmark_variables)}")
    print(f"  Poses: {sum(len(poses) for poses in fg_data.pose_variables)}")
    print(f"  Range measurements: {len(fg_data.range_measurements)}")

    print_worst_ranges(fg_data)
    plot_range_histogram(fg_data, save_dir=WORK_DIR)
    plot_range_diagnostics(fg_data, save_dir=WORK_DIR)
    plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True,
                         save_dir=WORK_DIR)
