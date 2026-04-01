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

from __future__ import annotations

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from copy import deepcopy

import gtsam
import numpy as np
from evo.core.trajectory import PosePath3D
from evo.core import metrics
from evo.tools import plot as evo_plot

from py_factor_graph.factor_graph import FactorGraphData
from py_factor_graph.modifiers import make_all_ranges_perfect
from py_factor_graph.variables import PoseVariable3D, LandmarkVariable3D


def _name_to_key(name: str) -> int:
    """Map a PyFG variable name (e.g. 'A0', 'L5') to a GTSAM symbol key.

    "A0" -> gtsam.symbol('A', 0),  "B3" -> gtsam.symbol('B', 3)
    "L0" -> gtsam.symbol('L', 0)
    """
    return gtsam.symbol(name[0], int(name[1:]))


def _rot3(matrix: np.ndarray) -> gtsam.Rot3:
    """Build a GTSAM Rot3. Forces contiguous float64 to avoid gtsam 4.2 segfaults."""
    return gtsam.Rot3(np.ascontiguousarray(matrix, dtype=np.float64))


def _pose3_from_pyfg(pose: PoseVariable3D) -> gtsam.Pose3:
    """Convert PyFG PoseVariable3D to GTSAM Pose3."""
    return gtsam.Pose3(_rot3(pose.rotation_matrix),
                       np.array(pose.true_position, dtype=np.float64))


def _position_array_from_pyfg(landmark: LandmarkVariable3D) -> np.ndarray:
    """Convert PyFG LandmarkVariable3D to numpy position array."""
    return np.array(landmark.true_position, dtype=np.float64)


def _pose3_noise(translation_precision: float, rotation_precision: float):
    """6-DOF diagonal noise from PyFG precisions.

    PyFG info matrix ordering (matrix_utils.py):
        3D -> diag([trans_prec]*3 + [2*rot_prec]*3)  -- translation first

    GTSAM Pose3 tangent ordering:
        [rot_x, rot_y, rot_z, tx, ty, tz]            -- rotation first

    Applies the 2x rotation factor from PyFG 3D and reorders to
    GTSAM's [rot, trans] convention.
    """
    rot_sigma = 1.0 / np.sqrt(2.0 * rotation_precision)
    trans_sigma = 1.0 / np.sqrt(translation_precision)
    sigmas = np.array([rot_sigma] * 3 + [trans_sigma] * 3)
    return gtsam.noiseModel.Diagonal.Sigmas(sigmas)


def _point3_noise(translation_precision: float):
    """3-DOF isotropic noise from PyFG translation precision."""
    return gtsam.noiseModel.Isotropic.Sigma(3, 1.0 / np.sqrt(translation_precision))


def _range_noise(stddev: float):
    """1-DOF range noise model."""
    return gtsam.noiseModel.Isotropic.Sigma(1, stddev)


def _odom_to_pose3(odom) -> gtsam.Pose3:
    """Convert PyFG PoseMeasurement3D to a relative GTSAM Pose3."""
    return gtsam.Pose3(_rot3(odom.rotation),
                       np.array([float(odom.x), float(odom.y), float(odom.z)],
                                dtype=np.float64))


def extract_trajectory(values: gtsam.Values,
                       pose_keys: list[int],
                       name: str = "") -> PosePath3D:
    """Build an evo PosePath3D from GTSAM Values for a list of pose keys."""
    matrices = []
    for key in pose_keys:
        pose: gtsam.Pose3 = values.atPose3(key)
        matrices.append(pose.matrix())
    return PosePath3D(poses_se3=matrices)


class FactorGraphSolver:
    """Builds a GTSAM factor graph from PyFG data and solves it.

    After construction, graph, initial, key_map, and gt_values are public
    attributes that can be modified directly before calling solve().
    """

    def __init__(self, fg: FactorGraphData, *,
                 use_odom_initial: bool = False,
                 use_true_ranges: bool = False,
                 odom_noise_sigmas: np.ndarray | None = None):
        """
        Args:
            fg:                 Source PyFG data (must be 3D).
            use_odom_initial:   Dead-reckon from odometry instead of ground truth.
            use_true_ranges:    Replace measured ranges with ground-truth distances.
            odom_noise_sigmas:  6-element stddev array in GTSAM Pose3 tangent order:
                                [rot_x, rot_y, rot_z, tx, ty, tz].
                                Perturbs both BetweenFactors and initial estimate.
        """
        if fg.dimension != 3:
            raise ValueError(f"Expected 3D factor graph, got {fg.dimension}D")

        self.fg = fg
        self.result: gtsam.Values | None = None

        self.key_map: dict[str, int] = {}
        for pose_chain in fg.pose_variables:
            for pose in pose_chain:
                self.key_map[pose.name] = _name_to_key(pose.name)
        for landmark in fg.landmark_variables:
            self.key_map[landmark.name] = _name_to_key(landmark.name)

        self._odom_deltas: list[list[gtsam.Pose3]] | None = None
        if odom_noise_sigmas is not None:
            odom_noise_sigmas = np.asarray(odom_noise_sigmas, dtype=np.float64)
            if odom_noise_sigmas.shape != (6,):
                raise ValueError("odom_noise_sigmas must have shape (6,)")
            self._odom_deltas = self._perturb_odom_deltas(odom_noise_sigmas)

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        self._build_initial(use_odom_initial)
        self._build_graph(use_true_ranges)

        self.gt_values = self._build_gt_values()

    def _perturb_odom_deltas(self, sigmas: np.ndarray) -> list[list[gtsam.Pose3]]:
        """Sample noisy odom deltas via Pose3 tangent-space perturbation.

        For each measurement, samples xi ~ N(0, diag(sigmas^2)) in
        [rot_x, rot_y, rot_z, tx, ty, tz], then composes:
            noisy_delta = clean_delta.compose(Pose3.Expmap(xi))
        """
        rng = np.random.default_rng(seed=42)
        noisy_deltas: list[list[gtsam.Pose3]] = []
        for odom_chain in self.fg.odom_measurements:
            chain: list[gtsam.Pose3] = []
            for odom in odom_chain:
                clean = _odom_to_pose3(odom)
                xi = rng.normal(0.0, sigmas)
                noise_pose = gtsam.Pose3.Expmap(xi)
                chain.append(clean.compose(noise_pose))
            noisy_deltas.append(chain)
        return noisy_deltas

    def _get_odom_delta(self, chain_idx: int, meas_idx: int, odom) -> gtsam.Pose3:
        """Return odom delta, perturbed if noise was requested."""
        if self._odom_deltas is not None:
            return self._odom_deltas[chain_idx][meas_idx]
        return _odom_to_pose3(odom)

    def _build_initial(self, use_odom_initial: bool) -> None:
        """Populate self.initial with ground truth or dead-reckoned poses."""
        if use_odom_initial:
            for robot_idx, pose_chain in enumerate(self.fg.pose_variables):
                if not pose_chain:
                    continue
                first_key = self.key_map[pose_chain[0].name]
                current = _pose3_from_pyfg(pose_chain[0])
                self.initial.insert(first_key, current)

                for i, odom in enumerate(self.fg.odom_measurements[robot_idx]):
                    delta = self._get_odom_delta(robot_idx, i, odom)
                    current = current.compose(delta)
                    key = self.key_map[odom.to_pose]
                    if not self.initial.exists(key):
                        self.initial.insert(key, current)
        else:
            for pose_chain in self.fg.pose_variables:
                for pose in pose_chain:
                    self.initial.insert(self.key_map[pose.name],
                                        _pose3_from_pyfg(pose))

        for landmark in self.fg.landmark_variables:
            self.initial.insert(self.key_map[landmark.name],
                                _position_array_from_pyfg(landmark))

    def _build_graph(self, use_true_ranges: bool) -> None:
        """Populate self.graph with all factors."""
        fg = self.fg

        for prior in fg.pose_priors:
            key = self.key_map[prior.name]
            R = _rot3(prior.rotation_matrix)
            t = np.array(prior.position, dtype=np.float64)
            pose = gtsam.Pose3(R, t)
            noise = _pose3_noise(prior.translation_precision,
                                 prior.rotation_precision)
            self.graph.addPriorPose3(key, pose, noise)

        for prior in fg.landmark_priors:
            key = self.key_map[prior.name]
            point = np.array(prior.position, dtype=np.float64)
            noise = _point3_noise(prior.translation_precision)
            self.graph.addPriorPoint3(key, point, noise)

        for chain_idx, odom_chain in enumerate(fg.odom_measurements):
            for i, odom in enumerate(odom_chain):
                key_from = self.key_map[odom.base_pose]
                key_to = self.key_map[odom.to_pose]
                delta = self._get_odom_delta(chain_idx, i, odom)
                noise = _pose3_noise(odom.translation_precision,
                                     odom.rotation_precision)
                self.graph.add(
                    gtsam.BetweenFactorPose3(key_from, key_to, delta, noise))

        for lc in fg.loop_closure_measurements:
            key_from = self.key_map[lc.base_pose]
            key_to = self.key_map[lc.to_pose]
            R = _rot3(lc.rotation)
            t = np.array([float(lc.x), float(lc.y), float(lc.z)],
                         dtype=np.float64)
            delta = gtsam.Pose3(R, t)
            noise = _pose3_noise(lc.translation_precision, lc.rotation_precision)
            self.graph.add(
                gtsam.BetweenFactorPose3(key_from, key_to, delta, noise))

        pose_keys = fg.pose_variables_dict
        if use_true_ranges:
            true_fg = make_all_ranges_perfect(fg)
            range_source = true_fg.range_measurements
        else:
            range_source = fg.range_measurements

        for rm in range_source:
            name_a, name_b = rm.association
            if name_a not in self.key_map or name_b not in self.key_map:
                continue

            key_a = self.key_map[name_a]
            key_b = self.key_map[name_b]
            # noise = _range_noise(rm.stddev)
            noise = _range_noise(4.0)

            a_is_pose = name_a in pose_keys
            b_is_pose = name_b in pose_keys

            if a_is_pose and b_is_pose:
                self.graph.add(
                    gtsam.RangeFactorPose3(key_a, key_b, rm.dist, noise))
            elif a_is_pose:
                self.graph.add(
                    gtsam.RangeFactor3D(key_a, key_b, rm.dist, noise))
            elif b_is_pose:
                self.graph.add(
                    gtsam.RangeFactor3D(key_b, key_a, rm.dist, noise))
            else:
                self.graph.add(
                    gtsam.RangeFactor3(key_a, key_b, rm.dist, noise))

    def _build_gt_values(self) -> gtsam.Values:
        """Build GTSAM Values from ground-truth poses and landmarks."""
        gt = gtsam.Values()
        for pose_chain in self.fg.pose_variables:
            for pose in pose_chain:
                gt.insert(self.key_map[pose.name], _pose3_from_pyfg(pose))
        for landmark in self.fg.landmark_variables:
            gt.insert(self.key_map[landmark.name],
                      _position_array_from_pyfg(landmark))
        return gt

    def solve(self, params: gtsam.LevenbergMarquardtParams | None = None
              ) -> gtsam.Values:
        """Run LM optimizer. Returns and stores result as self.result."""
        if params is None:
            params = gtsam.LevenbergMarquardtParams()
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, deepcopy(self.initial), params)
        self.result = optimizer.optimize()
        return self.result


def visualize(solver: FactorGraphSolver):
    """Plot ground-truth, initial, and optimized trajectories with APE."""
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
    # FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/tests/debug_single_robot.pfg"

    print(f"Reading {FILE_PATH} ...")
    fg_data = read_from_pyfg_text(FILE_PATH)

    print(f"  {fg_data.dimension}D, {fg_data.num_poses} poses, "
          f"{fg_data.num_landmarks} landmarks, "
          f"{fg_data.num_odom_measurements} odom, "
          f"{len(fg_data.range_measurements)} range")

    # Run 1: measured ranges
    print("\n=== Run 1: Measured Ranges ===")
    solver_measured = FactorGraphSolver(fg_data, use_odom_initial=True)
    solver_measured.solve()
    print(f"GTSAM graph: {solver_measured.graph.size()} factors, "
          f"{solver_measured.initial.size()} variables")
    print(f"Initial error: {solver_measured.graph.error(solver_measured.initial):.4f}")
    print(f"Final   error: {solver_measured.graph.error(solver_measured.result):.4f}")

    # Run 2: true ranges
    print("\n=== Run 2: True Ranges ===")
    solver_true = FactorGraphSolver(fg_data, use_odom_initial=True,
                                    use_true_ranges=True)
    solver_true.solve()
    print(f"GTSAM graph: {solver_true.graph.size()} factors, "
          f"{solver_true.initial.size()} variables")
    print(f"Initial error: {solver_true.graph.error(solver_true.initial):.4f}")
    print(f"Final   error: {solver_true.graph.error(solver_true.result):.4f}")

    # Visualize both
    print("\n--- Measured Ranges ---")
    visualize(solver_measured)
    print("\n--- True Ranges ---")
    visualize(solver_true)
