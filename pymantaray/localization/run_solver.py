#!/usr/bin/env python3
"""Run the factor graph solver with measured and true ranges, then visualize."""

import os
from copy import deepcopy

from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, RobustConfig
from solver_defaults import build_default_config
from visualize_solver import (visualize, compare_results, compare_depth_error,
                               visualize_landmarks)
from debug_factor_graphs import debug_factor_graph

# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-simple/output.pfg"
# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-no-multi/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-fleet-week/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-simple/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-float/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
WORK_DIR = os.path.dirname(FILE_PATH)

print(f"Reading {FILE_PATH} ...")
fg_data = read_from_pyfg_text(FILE_PATH)

print(f"  {fg_data.dimension}D, {fg_data.num_poses} poses, "
      f"{fg_data.num_landmarks} landmarks, "
      f"{fg_data.num_odom_measurements} odom, "
      f"{len(fg_data.range_measurements)} range")

config = build_default_config(fg_data)

print("\n=== Run 1: Measured Ranges ===")
solver_measured = FactorGraphSolver(fg_data, config)
solver_measured.solve()
print(f"GTSAM graph: {solver_measured.graph.size()} factors, "
      f"{solver_measured.initial.size()} variables")
print(f"Initial error: {solver_measured.graph.error(solver_measured.initial):.4f}")
print(f"Final   error: {solver_measured.graph.error(solver_measured.result):.4f}")

# debug_factor_graph(solver_measured, save_dir=WORK_DIR, prefix="measured",
#                    show_3d=True, show_leverage=True)

print("\n--- Measured Ranges ---")
visualize(solver_measured, save_dir=WORK_DIR, prefix="measured",
          estimate_label="Refracted Ranges", show=False)
visualize_landmarks(solver_measured, save_dir=WORK_DIR, prefix="measured")

print("\n=== Run 2: True Ranges ===")
config_true = deepcopy(config)
config_true.use_true_ranges = True
solver_true = FactorGraphSolver(fg_data, config_true)
solver_true.solve()
print(f"GTSAM graph: {solver_true.graph.size()} factors, "
      f"{solver_true.initial.size()} variables")
print(f"Initial error: {solver_true.graph.error(solver_true.initial):.4f}")
print(f"Final   error: {solver_true.graph.error(solver_true.result):.4f}")

# debug_factor_graph(solver_true, save_dir=WORK_DIR, prefix="true",
#                    show_3d=False)

print("\n--- True Ranges ---")
visualize(solver_true, save_dir=WORK_DIR, prefix="true", show_range_error=False,
          estimate_label="Straight-line Ranges", show=False)
visualize_landmarks(solver_true, save_dir=WORK_DIR, prefix="true")


# print("\n=== Run 4: Robust Ranges ===")
# config_robust = deepcopy(config)
# config_robust.robust_range = RobustConfig(kernel="cauchy", param=100.0)
# solver_robust = FactorGraphSolver(fg_data, config_robust)
# solver_robust.solve()
# print(f"GTSAM graph: {solver_robust.graph.size()} factors, "
#       f"{solver_robust.initial.size()} variables")
# print(f"Initial error: {solver_robust.graph.error(solver_robust.initial):.4f}")
# print(f"Final   error: {solver_robust.graph.error(solver_robust.result):.4f}")
#
# print("\n--- Robust Ranges ---")
# visualize(solver_robust, save_dir=WORK_DIR, prefix="welsch_robust", show_range_error=False)
# visualize_landmarks(solver_robust, save_dir=WORK_DIR, prefix="welsch_robust")

print("\n=== Comparison ===")
compare_results(
    [
        # solver_no_range,
        solver_measured,
        solver_true,
        # solver_robust,
    ],
    [
        # "GPS + Depth",
        "Refracted Ranges",
        "Straight-line Ranges",
        # "Robust Refracted Ranges",
    ],
    save_dir=WORK_DIR,
    show_landmark_hull= True,
    show_topdown=True,
    show_xyz_stack=True,
)

