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

from copy import deepcopy
from dataclasses import dataclass, field

import gtsam
import numpy as np
from evo.core.trajectory import PosePath3D

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


_ROBUST_KERNELS = {
    "tukey": gtsam.noiseModel.mEstimator.Tukey.Create,
    "huber": gtsam.noiseModel.mEstimator.Huber.Create,
    "cauchy": gtsam.noiseModel.mEstimator.Cauchy.Create,
    "geman-mcclure": gtsam.noiseModel.mEstimator.GemanMcClure.Create,
    "welsch": gtsam.noiseModel.mEstimator.Welsch.Create,
}


@dataclass
class RobustConfig:
    """M-estimator configuration for robust noise models.

    Args:
        kernel: Which m-estimator kernel. Options: "tukey", "huber",
                "cauchy", "geman-mcclure", "welsch".
        param:  Kernel-specific threshold parameter
                (e.g. Tukey's c, Huber's k).
    """
    kernel: str = "tukey"
    param: float = 15.0

    def __post_init__(self):
        if self.kernel not in _ROBUST_KERNELS:
            raise ValueError(
                f"Unknown robust kernel '{self.kernel}', "
                f"choose from {list(_ROBUST_KERNELS.keys())}")


def _range_noise(stddev: float,
                 robust: RobustConfig | None = None):
    """1-DOF range noise model, optionally wrapped with a robust m-estimator."""
    gaussian = gtsam.noiseModel.Isotropic.Sigma(1, stddev)
    if robust is None:
        return gaussian
    return gtsam.noiseModel.Robust.Create(
        _ROBUST_KERNELS[robust.kernel](robust.param), gaussian)


def _odom_to_pose3(odom) -> gtsam.Pose3:
    """Convert PyFG PoseMeasurement3D to a relative GTSAM Pose3."""
    return gtsam.Pose3(_rot3(odom.rotation),
                       np.array([odom.x, odom.y, odom.z],
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


@dataclass
class SolverConfig:
    """Configuration for FactorGraphSolver.

    Args:
        use_odom_initial:    Dead-reckon from odometry instead of ground truth.
        use_true_ranges:     Replace measured ranges with ground-truth distances.
        include_gps_priors:  Include GPS pose priors (all except first-pose priors).
        include_ranges:      Include range measurement factors.
        range_noise_stddev:  Stddev (meters) for range factor noise model.
        odom_noise_sigmas:   6-element stddev array in GTSAM Pose3 tangent order:
                                 [rot_x (rad), rot_y (rad), rot_z (rad),
                                  tx (m), ty (m), tz (m)]
                             When set, BetweenFactors use Expmap-perturbed deltas
                             and initial estimate defaults to ground truth.
        between_noise_sigmas: 6-element stddev array for BetweenFactorPose3
                             noise model, same GTSAM Pose3 tangent ordering.
                             When set, overrides PyFG odometry precisions.
                             When None, falls back to PyFG precisions.
        robust_range:        RobustConfig for range noise m-estimator.
                             None = standard Gaussian (no robustness).
        landmark_prior_sigma: Isotropic stddev (meters) for landmark priors.
                             When None, falls back to PyFG precisions.
        seed:                RNG seed for all random number generation
                             (odom perturbation).
    """
    use_odom_initial: bool = False
    use_true_ranges: bool = False
    include_gps_priors: bool = True
    include_ranges: bool = True
    range_noise_stddev: float = 4.0
    robust_range: RobustConfig | None = None
    landmark_prior_sigma: float | None = None
    odom_noise_sigmas: np.ndarray | None = field(default=None, repr=False)
    between_noise_sigmas: np.ndarray | None = field(default=None, repr=False)
    seed: int = 42

    def __post_init__(self):
        if self.odom_noise_sigmas is not None:
            self.odom_noise_sigmas = np.asarray(
                self.odom_noise_sigmas, dtype=np.float64).flatten()
            if self.odom_noise_sigmas.shape != (6,):
                raise ValueError(
                    f"odom_noise_sigmas must have 6 elements, "
                    f"got shape {self.odom_noise_sigmas.shape}")
        if self.between_noise_sigmas is not None:
            self.between_noise_sigmas = np.asarray(
                self.between_noise_sigmas, dtype=np.float64).flatten()
            if self.between_noise_sigmas.shape != (6,):
                raise ValueError(
                    f"between_noise_sigmas must have 6 elements, "
                    f"got shape {self.between_noise_sigmas.shape}")
        if self.range_noise_stddev <= 0:
            raise ValueError(
                f"range_noise_stddev must be positive, got {self.range_noise_stddev}")
        if self.landmark_prior_sigma is not None and self.landmark_prior_sigma <= 0:
            raise ValueError(
                f"landmark_prior_sigma must be positive, "
                f"got {self.landmark_prior_sigma}")


class FactorGraphSolver:
    """Builds a GTSAM factor graph from PyFG data and solves it.

    After construction, graph, initial, key_map, and gt_values are public
    attributes that can be modified directly before calling solve().
    """

    def __init__(self, fg: FactorGraphData,
                 config: SolverConfig | None = None):
        """
        Args:
            fg:     Source PyFG data (must be 3D).
            config: Solver configuration. Uses defaults if not provided.
        """
        if fg.dimension != 3:
            raise ValueError(f"Expected 3D factor graph, got {fg.dimension}D")

        self.fg = fg
        self.config = config or SolverConfig()
        self.result: gtsam.Values | None = None

        self.key_map: dict[str, int] = {}
        for pose_chain in fg.pose_variables:
            for pose in pose_chain:
                self.key_map[pose.name] = _name_to_key(pose.name)
        for landmark in fg.landmark_variables:
            self.key_map[landmark.name] = _name_to_key(landmark.name)

        self._odom_deltas: list[list[gtsam.Pose3]] | None = None
        if self.config.odom_noise_sigmas is not None:
            self._odom_deltas = self._perturb_odom_deltas(
                self.config.odom_noise_sigmas)

        self.graph = gtsam.NonlinearFactorGraph()
        self.initial = gtsam.Values()

        self._build_initial(self.config.use_odom_initial)
        self._build_graph()

        self.gt_values = self._build_gt_values()
        self.odom_values = self._build_odom_values()

    def _perturb_odom_deltas(self, sigmas: np.ndarray) -> list[list[gtsam.Pose3]]:
        """Sample noisy odom deltas via Pose3 tangent-space perturbation.

        For each measurement, samples xi ~ N(0, diag(sigmas^2)) in
        [rot_x, rot_y, rot_z, tx, ty, tz], then composes:
            noisy_delta = clean_delta.compose(Pose3.Expmap(xi))
        """
        rng = np.random.default_rng(seed=self.config.seed)
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
        """Populate self.initial with ground truth or dead-reckoned poses.

        When odom_noise_sigmas is set, initial is always ground truth
        (noise only affects BetweenFactors in the graph).
        """
        if use_odom_initial and self._odom_deltas is None:
            for robot_idx, pose_chain in enumerate(self.fg.pose_variables):
                if not pose_chain:
                    continue
                first_key = self.key_map[pose_chain[0].name]
                current = _pose3_from_pyfg(pose_chain[0])
                self.initial.insert(first_key, current)

                for i, odom in enumerate(self.fg.odom_measurements[robot_idx]):
                    delta = _odom_to_pose3(odom)
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

    def _build_graph(self) -> None:
        """Populate self.graph with all factors."""
        fg = self.fg
        cfg = self.config

        first_pose_names = set()
        for pose_chain in fg.pose_variables:
            if pose_chain:
                first_pose_names.add(pose_chain[0].name)

        for prior in fg.pose_priors:
            is_first_pose = prior.name in first_pose_names
            if not is_first_pose and not cfg.include_gps_priors:
                continue
            key = self.key_map[prior.name]
            R = _rot3(prior.rotation_matrix)
            t = np.array(prior.position, dtype=np.float64)
            pose = gtsam.Pose3(R, t)
            noise = _pose3_noise(prior.translation_precision,
                                 prior.rotation_precision)
            self.graph.addPriorPose3(key, pose, noise)

        landmark_noise = (gtsam.noiseModel.Isotropic.Sigma(3, cfg.landmark_prior_sigma)
                          if cfg.landmark_prior_sigma is not None else None)

        for prior in fg.landmark_priors:
            key = self.key_map[prior.name]
            point = np.array(prior.position, dtype=np.float64)
            noise = (landmark_noise if landmark_noise is not None
                     else _point3_noise(prior.translation_precision))
            self.graph.addPriorPoint3(key, point, noise)

        between_noise = (gtsam.noiseModel.Diagonal.Sigmas(cfg.between_noise_sigmas)
                         if cfg.between_noise_sigmas is not None else None)

        for chain_idx, odom_chain in enumerate(fg.odom_measurements):
            for i, odom in enumerate(odom_chain):
                key_from = self.key_map[odom.base_pose]
                key_to = self.key_map[odom.to_pose]
                delta = self._get_odom_delta(chain_idx, i, odom)
                noise = (between_noise if between_noise is not None
                         else _pose3_noise(odom.translation_precision,
                                           odom.rotation_precision))
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

        if not cfg.include_ranges:
            return

        pose_keys = fg.pose_variables_dict
        if cfg.use_true_ranges:
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
            noise = _range_noise(cfg.range_noise_stddev, cfg.robust_range)

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

    def _build_odom_values(self) -> gtsam.Values:
        """Build GTSAM Values by dead-reckoning from (possibly perturbed) odom deltas.

        First pose per robot uses ground truth, then composes odom deltas.
        When odom_noise_sigmas is set, this shows the perturbed trajectory.
        """
        odom = gtsam.Values()
        for robot_idx, pose_chain in enumerate(self.fg.pose_variables):
            if not pose_chain:
                continue
            current = _pose3_from_pyfg(pose_chain[0])
            odom.insert(self.key_map[pose_chain[0].name], current)

            for i, meas in enumerate(self.fg.odom_measurements[robot_idx]):
                delta = self._get_odom_delta(robot_idx, i, meas)
                current = current.compose(delta)
                key = self.key_map[meas.to_pose]
                if not odom.exists(key):
                    odom.insert(key, current)

        for landmark in self.fg.landmark_variables:
            odom.insert(self.key_map[landmark.name],
                        _position_array_from_pyfg(landmark))
        return odom

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
