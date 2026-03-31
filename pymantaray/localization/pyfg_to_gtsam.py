#!/usr/bin/env python3
"""
Convert a PyFactorGraph FactorGraphData into a GTSAM NonlinearFactorGraph + Values.

Supports 3D factor graphs with:
  - Pose variables (SE3)
  - Landmark variables (Point3)
  - Odometry / loop closure factors (BetweenFactorPose3)
  - Range factors (pose-pose, pose-landmark)
  - Pose priors
  - Landmark priors
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import gtsam
import numpy as np
from evo.core.trajectory import PosePath3D
from evo.core import metrics
from evo.tools import plot as evo_plot

from py_factor_graph.factor_graph import FactorGraphData
from py_factor_graph.variables import PoseVariable3D, LandmarkVariable3D


def _name_to_key(name: str) -> int:
    """Map a PyFG variable name (e.g. 'A0', 'L5') to a GTSAM symbol key.

    Poses use the robot letter as the symbol character:
        "A0" -> gtsam.symbol('A', 0),  "B3" -> gtsam.symbol('B', 3)
    Landmarks use 'L':
        "L0" -> gtsam.symbol('L', 0)
    """
    char = name[0]
    idx = int(name[1:])
    return gtsam.symbol(char, idx)


def _rot3(matrix: np.ndarray) -> gtsam.Rot3:
    """Build a GTSAM Rot3, ensuring the matrix is contiguous float64.

    gtsam 4.2's pybind Rot3(ndarray) segfaults on non-contiguous or
    non-float64 arrays (and on numpy >= 2.0).
    """
    return gtsam.Rot3(np.ascontiguousarray(matrix, dtype=np.float64))


def _pose3_from_pyfg(pose: PoseVariable3D) -> gtsam.Pose3:
    """Build a GTSAM Pose3 from a PyFG PoseVariable3D."""
    R = _rot3(pose.rotation_matrix)
    t = np.array(pose.true_position, dtype=np.float64)
    return gtsam.Pose3(R, t)


def _point3_from_pyfg(landmark: LandmarkVariable3D) -> np.ndarray:
    """Build a GTSAM Point3 (ndarray) from a PyFG LandmarkVariable3D."""
    return np.array(landmark.true_position, dtype=np.float64)


def _pose3_noise(translation_precision: float, rotation_precision: float):
    """Create a 6-DOF diagonal noise model for Pose3 factors.

    PyFG info matrix ordering (matrix_utils.py):
        3D -> diag([trans_prec]*3 + [2*rot_prec]*3)  -- translation first

    GTSAM Pose3 tangent ordering:
        [rot_x, rot_y, rot_z, tx, ty, tz]            -- rotation first

    This function applies the 2x rotation factor that PyFG uses in 3D
    and reorders to GTSAM's [rot, trans] convention.
    """
    rot_sigma = 1.0 / np.sqrt(2.0 * rotation_precision)
    trans_sigma = 1.0 / np.sqrt(translation_precision)
    sigmas = np.array([rot_sigma] * 3 + [trans_sigma] * 3)
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


def _point3_noise(translation_precision: float):
    """Create a 3-DOF isotropic noise model for Point3 factors."""
    sigma = 1.0 / np.sqrt(translation_precision)
    return gtsam.noiseModel.Isotropic.Sigma(3, sigma)


def _range_noise(stddev: float):
    """Create a 1-DOF noise model for range factors."""
    return gtsam.noiseModel.Isotropic.Sigma(1, stddev)


def _odom_to_pose3(odom) -> gtsam.Pose3:
    """Build a relative GTSAM Pose3 from a PyFG PoseMeasurement3D."""
    R = _rot3(odom.rotation)
    t = np.array([float(odom.x), float(odom.y), float(odom.z)],
                 dtype=np.float64)
    return gtsam.Pose3(R, t)


def convert(fg: FactorGraphData, use_odom_initial: bool = False):
    """Convert a 3D PyFactorGraph to GTSAM.

    Args:
        fg:                 PyFG FactorGraphData (must be 3D)
        use_odom_initial:   If True, initialize pose estimates by composing
                            odometry (dead reckoning) instead of ground truth.
                            The first pose per robot still uses ground truth.

    Returns:
        graph:          gtsam.NonlinearFactorGraph
        initial:        gtsam.Values
        key_map:        dict mapping PyFG variable name -> GTSAM key (int)
    """
    if fg.dimension != 3:
        raise ValueError(f"Expected 3D factor graph, got {fg.dimension}D")

    graph = gtsam.NonlinearFactorGraph()
    initial = gtsam.Values()
    key_map: dict[str, int] = {}

    for pose_chain in fg.pose_variables:
        for pose in pose_chain:
            key = _name_to_key(pose.name)
            key_map[pose.name] = key

    if use_odom_initial:
        for robot_idx, pose_chain in enumerate(fg.pose_variables):
            if not pose_chain:
                continue
            first_key = key_map[pose_chain[0].name]
            current = _pose3_from_pyfg(pose_chain[0])
            initial.insert(first_key, current)

            for odom in fg.odom_measurements[robot_idx]:
                current = current.compose(_odom_to_pose3(odom))
                key = key_map[odom.to_pose]
                if not initial.exists(key):
                    initial.insert(key, current)
    else:
        for pose_chain in fg.pose_variables:
            for pose in pose_chain:
                initial.insert(key_map[pose.name], _pose3_from_pyfg(pose))

    for landmark in fg.landmark_variables:
        key = _name_to_key(landmark.name)
        key_map[landmark.name] = key
        initial.insert(key, _point3_from_pyfg(landmark))

    for prior in fg.pose_priors:
        key = key_map[prior.name]
        R = _rot3(prior.rotation_matrix)
        t = np.array(prior.position, dtype=np.float64)
        pose = gtsam.Pose3(R, t)
        noise = _pose3_noise(prior.translation_precision, prior.rotation_precision)
        graph.addPriorPose3(key, pose, noise)

    for prior in fg.landmark_priors:
        key = key_map[prior.name]
        point = np.array(prior.position, dtype=np.float64)
        noise = _point3_noise(prior.translation_precision)
        graph.addPriorPoint3(key, point, noise)

    for odom_chain in fg.odom_measurements:
        for odom in odom_chain:
            key_from = key_map[odom.base_pose]
            key_to = key_map[odom.to_pose]
            R = _rot3(odom.rotation)
            t = np.array([float(odom.x), float(odom.y), float(odom.z)],
                         dtype=np.float64)
            delta = gtsam.Pose3(R, t)
            noise = _pose3_noise(odom.translation_precision, odom.rotation_precision)
            graph.add(gtsam.BetweenFactorPose3(key_from, key_to, delta, noise))

    for lc in fg.loop_closure_measurements:
        key_from = key_map[lc.base_pose]
        key_to = key_map[lc.to_pose]
        R = _rot3(lc.rotation)
        t = np.array([float(lc.x), float(lc.y), float(lc.z)],
                     dtype=np.float64)
        delta = gtsam.Pose3(R, t)
        noise = _pose3_noise(lc.translation_precision, lc.rotation_precision)
        graph.add(gtsam.BetweenFactorPose3(key_from, key_to, delta, noise))

    pose_keys = fg.pose_variables_dict
    for rm in fg.range_measurements:
        """Range factor type depends on endpoint variable types:
            RangeFactorPose3:  Pose3 <-> Pose3
            RangeFactor3D:     Pose3 <-> Point3
            RangeFactor3:      Point3 <-> Point3
        """
        name_a, name_b = rm.association
        if name_a not in key_map or name_b not in key_map:
            continue

        key_a = key_map[name_a]
        key_b = key_map[name_b]
        noise = _range_noise(rm.stddev)

        a_is_pose = name_a in pose_keys
        b_is_pose = name_b in pose_keys

        if a_is_pose and b_is_pose:
            graph.add(gtsam.RangeFactorPose3(key_a, key_b, rm.dist, noise))
        elif a_is_pose:
            graph.add(gtsam.RangeFactor3D(key_a, key_b, rm.dist, noise))
        elif b_is_pose:
            graph.add(gtsam.RangeFactor3D(key_b, key_a, rm.dist, noise))
        else:
            graph.add(gtsam.RangeFactor3(key_a, key_b, rm.dist, noise))

    return graph, initial, key_map


def _values_to_pose_list(values: gtsam.Values,
                         pose_keys: list[int]) -> list[np.ndarray]:
    """Extract ordered 4x4 SE3 matrices from GTSAM Values for the given keys."""
    matrices = []
    for key in pose_keys:
        pose: gtsam.Pose3 = values.atPose3(key)
        matrices.append(pose.matrix())
    return matrices


def extract_trajectory(values: gtsam.Values,
                       pose_keys: list[int],
                       name: str = "") -> PosePath3D:
    """Build an evo PosePath3D from GTSAM Values for a list of pose keys."""
    matrices = _values_to_pose_list(values, pose_keys)
    return PosePath3D(poses_se3=matrices)


def _build_gt_values(fg: FactorGraphData, key_map: dict[str, int]) -> gtsam.Values:
    """Build a GTSAM Values populated with ground-truth poses and landmarks."""
    gt = gtsam.Values()
    for pose_chain in fg.pose_variables:
        for pose in pose_chain:
            gt.insert(key_map[pose.name], _pose3_from_pyfg(pose))
    for landmark in fg.landmark_variables:
        gt.insert(key_map[landmark.name], _point3_from_pyfg(landmark))
    return gt


def visualize(fg: FactorGraphData,
              graph: gtsam.NonlinearFactorGraph,
              initial: gtsam.Values,
              result: gtsam.Values,
              key_map: dict[str, int]):
    """Plot ground-truth, odometry initial, and optimized trajectories with APE.

    Shows per robot:
      1. 3D trajectory overlay (ground-truth / dead-reckoning initial / optimized)
      2. APE for both the initial estimate and the optimized result
    """
    gt_values = _build_gt_values(fg, key_map)

    for robot_idx, pose_chain in enumerate(fg.pose_variables):
        if not pose_chain:
            continue

        keys_ordered = [key_map[p.name] for p in pose_chain]
        robot_char = pose_chain[0].name[0]

        traj_gt = extract_trajectory(gt_values, keys_ordered)
        traj_init = extract_trajectory(initial, keys_ordered)
        traj_opt = extract_trajectory(result, keys_ordered)

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
                      style='--', color='red', label='Dead Reckoning (initial)')
        evo_plot.traj(ax, evo_plot.PlotMode.xyz, traj_opt,
                      style='-', color='blue', label='Optimized')
        ax.legend()
        ax.set_title(f"Robot {robot_char} — Trajectory Comparison")

        fig2 = plt.figure(figsize=(10, 4))
        ax2 = fig2.add_subplot(111)
        ax2.plot(ape_init.error, color='red', linewidth=0.8,
                 alpha=0.6, label='Initial (dead reckoning)')
        ax2.plot(ape_opt.error, color='blue', linewidth=0.8,
                 label='Optimized')
        ax2.set_xlabel("Pose index")
        ax2.set_ylabel("APE (m)")
        ax2.set_title(f"Robot {robot_char} — Absolute Pose Error")
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        fig2.tight_layout()

    plt.show()


if __name__ == "__main__":
    from py_factor_graph.io.pyfg_text import read_from_pyfg_text

    FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src/output.pfg"

    print(f"Reading {FILE_PATH} ...")
    fg_data = read_from_pyfg_text(FILE_PATH)

    print(f"  {fg_data.dimension}D, {fg_data.num_poses} poses, "
          f"{fg_data.num_landmarks} landmarks, "
          f"{fg_data.num_odom_measurements} odom, "
          f"{len(fg_data.range_measurements)} range")

    graph, initial, key_map = convert(fg_data, use_odom_initial=True)

    print(f"\nGTSAM graph: {graph.size()} factors, {initial.size()} variables")

    params = gtsam.LevenbergMarquardtParams()
    optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
    result = optimizer.optimize()

    print(f"Initial error: {graph.error(initial):.4f}")
    print(f"Final   error: {graph.error(result):.4f}")

    visualize(fg_data, graph, initial, result, key_map)
