#!/usr/bin/env python3
"""
Simple script to read a .pyfg file and plot 3D values.
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Import parsing function
from py_factor_graph.io.pyfg_text import read_from_pyfg_text
from py_factor_graph.utils.plot_utils import (
    draw_pose_3d,
    draw_landmark_variable_3d,
    draw_traj_3d,
    draw_range_measurement_3d,
)
from py_factor_graph.variables import PoseVariable3D


# ============ CONFIGURATION ============
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src/output.pfg"  # <-- SET YOUR FILE PATH HERE


# ======================================


def plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True):
    """
    Plot 3D factor graph data.

    Args:
        fg_data: FactorGraphData object (must be 3D)
        show_trajectories: Whether to plot pose trajectories
        show_landmarks: Whether to plot landmark variables
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

    # plt.tight_layout()
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

    # Plot the data
    plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True)
