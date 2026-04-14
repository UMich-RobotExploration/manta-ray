#!/usr/bin/env python3
"""Run the factor graph solver with measured and true ranges, then visualize."""

import os
from copy import deepcopy

import numpy as np
from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, SolverConfig, RobustConfig
from visualize_solver import (visualize, compare_results, compare_depth_error,
                               visualize_landmarks)

# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-simple/output.pfg"
# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-no-multi/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-simple/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-float/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
WORK_DIR = os.path.dirname(FILE_PATH)

# Odom noise fractions of motion (per-component) in GTSAM Pose3 tangent order:
#   [rot_x, rot_y, rot_z, tx, ty, tz]
# Per delta, sigma_i = max(floor, frac_i * |motion_i|).
# Translation: 5% of distance traveled on x/y; z drift is much smaller.
default_pos_prior = 0.1 # meters

angular_noise = 1E-6 # radians
xy_frac = 0.05
z_frac = 0.01
odom_noise = np.array(
    [angular_noise, angular_noise, angular_noise,
     xy_frac, xy_frac, z_frac])

odom_gtsam_noise = deepcopy(odom_noise)
odom_gtsam_noise[:3] = 1E-2   # factor-graph rotation belief (rad-per-rad)
# odom_gtsam_noise[2] = 0.10
# Translation fractions stay equal to odom_noise (matched factor belief).

# GPS prior: position stddev 0.1 m, rotation stddev 2 rad (effectively unconstrained).
# Order is Pose3 tangent [rot_x, rot_y, rot_z, tx, ty, tz].
gps_prior_sigmas = np.array([2, 2, 2, default_pos_prior, default_pos_prior, default_pos_prior],
                            dtype=np.float64)

depth_prior_sigma = 0.01/3.0  # meters — pressure-sensor stddev

config = SolverConfig(
    odom_noise_sigmas=odom_noise,
    range_noise_stddev=1.0,
    include_ranges=True,
    between_noise_sigmas=odom_gtsam_noise,
    landmark_prior_sigma=default_pos_prior,
    gps_prior_sigmas=gps_prior_sigmas,
    depth_prior_sigma=depth_prior_sigma,
    depth_prior_mode="custom",
)

print(f"Reading {FILE_PATH} ...")
fg_data = read_from_pyfg_text(FILE_PATH)

print(f"  {fg_data.dimension}D, {fg_data.num_poses} poses, "
      f"{fg_data.num_landmarks} landmarks, "
      f"{fg_data.num_odom_measurements} odom, "
      f"{len(fg_data.range_measurements)} range")

print("\n=== Run 1: Measured Ranges ===")
solver_measured = FactorGraphSolver(fg_data, config)
solver_measured.solve()
print(f"GTSAM graph: {solver_measured.graph.size()} factors, "
      f"{solver_measured.initial.size()} variables")
print(f"Initial error: {solver_measured.graph.error(solver_measured.initial):.4f}")
print(f"Final   error: {solver_measured.graph.error(solver_measured.result):.4f}")

print("\n--- Measured Ranges ---")
visualize(solver_measured, save_dir=WORK_DIR, prefix="measured",
          estimate_label="Ray-Traced Ranges")
visualize_landmarks(solver_measured, save_dir=WORK_DIR, prefix="measured")

print("\n=== Run 2: True Ranges ===")
config_true = deepcopy(config)
config_true.use_true_ranges = True
config_true.depth_prior_mode = "custom"
solver_true = FactorGraphSolver(fg_data, config_true)
solver_true.solve()
print(f"GTSAM graph: {solver_true.graph.size()} factors, "
      f"{solver_true.initial.size()} variables")
print(f"Initial error: {solver_true.graph.error(solver_true.initial):.4f}")
print(f"Final   error: {solver_true.graph.error(solver_true.result):.4f}")

print("\n--- True Ranges ---")
visualize(solver_true, save_dir=WORK_DIR, prefix="true", show_range_error=False,
          estimate_label="Idealized Ranges")
visualize_landmarks(solver_true, save_dir=WORK_DIR, prefix="true")

print("\n=== Run 3: Measured Ranges ===")
config_custom_depth = deepcopy(config)
config_custom_depth.depth_prior_mode = "custom"
solver_custom_depth = FactorGraphSolver(fg_data, config_custom_depth)
solver_custom_depth.solve()
print(f"GTSAM graph: {solver_custom_depth.graph.size()} factors, "
      f"{solver_custom_depth.initial.size()} variables")
print(f"Initial error: {solver_custom_depth.graph.error(solver_custom_depth.initial):.4f}")
print(f"Final   error: {solver_custom_depth.graph.error(solver_custom_depth.result):.4f}")

print("\n=== Run 4: GPS + Depth (no ranging) ===")
config_no_range = deepcopy(config)
config_no_range.depth_prior_mode = "custom"
config_no_range.include_ranges = False
solver_no_range = FactorGraphSolver(fg_data, config_no_range)
solver_no_range.solve()
print(f"GTSAM graph: {solver_no_range.graph.size()} factors, "
      f"{solver_no_range.initial.size()} variables")
print(f"Initial error: {solver_no_range.graph.error(solver_no_range.initial):.4f}")
print(f"Final   error: {solver_no_range.graph.error(solver_no_range.result):.4f}")

# print("\n=== Run 4: Robust Ranges ===")
# config_robust = deepcopy(config)
# config_robust.depth_prior_mode= "custom"
# config_robust.robust_range = RobustConfig(kernel="welsch", param=15.0)
# solver_robust = FactorGraphSolver(fg_data, config_robust)
# solver_robust.solve()
# print(f"GTSAM graph: {solver_robust.graph.size()} factors, "
#       f"{solver_robust.initial.size()} variables")
# print(f"Initial error: {solver_robust.graph.error(solver_robust.initial):.4f}")
# print(f"Final   error: {solver_robust.graph.error(solver_robust.result):.4f}")

# print("\n--- Robust Ranges ---")
# visualize(solver_robust, save_dir=WORK_DIR, prefix="welsch_robust", show_range_error=False)
# visualize_landmarks(solver_robust, save_dir=WORK_DIR, prefix="welsch_robust")

print("\n=== Comparison ===")
compare_results(
    [
        solver_no_range,
        solver_custom_depth,
        solver_true,
    ],
    [
        "GPS + Depth",
        "Ray-Traced Ranges",
        "Idealized Ranges",
    ],
    save_dir=WORK_DIR,
)

compare_depth_error(
    [
        solver_no_range,
        solver_custom_depth,
        solver_true,
    ],
    [
        "GPS + Depth",
        "Ray-Traced Ranges",
        "Idealized Ranges",
    ],
    save_dir=WORK_DIR,
)
