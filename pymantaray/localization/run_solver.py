#!/usr/bin/env python3
"""Run the factor graph solver with measured and true ranges, then visualize."""

from copy import deepcopy

import numpy as np
from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, SolverConfig
from visualize_solver import visualize

FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/tests/debug_single_robot.pfg"

# Odom perturbation stddevs in GTSAM Pose3 tangent order:
#   [rot_x (rad), rot_y (rad), rot_z (rad), tx (m), ty (m), tz (m)]
angular_noise = np.radians(1 / 3)
positional_noise = 0.1 * 1 / 3
odom_noise = np.array(
    [angular_noise, angular_noise, angular_noise,
     positional_noise, positional_noise, positional_noise])

config = SolverConfig(
    odom_noise_sigmas=odom_noise,
    range_noise_stddev=1.0,
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

print("\n=== Run 2: True Ranges ===")
config_true = deepcopy(config)
config_true.use_true_ranges = True
solver_true = FactorGraphSolver(fg_data, config_true)
solver_true.solve()
print(f"GTSAM graph: {solver_true.graph.size()} factors, "
      f"{solver_true.initial.size()} variables")
print(f"Initial error: {solver_true.graph.error(solver_true.initial):.4f}")
print(f"Final   error: {solver_true.graph.error(solver_true.result):.4f}")

print("\n--- Measured Ranges ---")
visualize(solver_measured)
print("\n--- True Ranges ---")
visualize(solver_true)
