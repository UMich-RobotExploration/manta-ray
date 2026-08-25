#!/usr/bin/env python3
"""MC trajectory runner: solve the factor graph once per seed and cache
the per-robot solved position tracks alongside the ground truth.

Mirrors the ``mc_paired_ape.py`` pattern (runner + plotter split) so we
can iterate on plot styling without re-running the LM optimizer. Only
the refracted (bellhop-measured) leg is stored -- that is the paper's
"real world" trajectory. Add ``use_true_ranges=True`` in a second pass
if you also want the idealized track cached.

Output ``mc_trajectories.npz`` (next to the pfg) contains
  seeds                     (n_seeds,)   int64
  gt_<r>                    (n_poses, 3) float64  -- ground truth positions
  odom_<r>                  (n_poses, 3) float64  -- odometry-only dead reckon
  mc_<r>                    (n_seeds, n_poses, 3) float64 -- solved positions

Edit ``FILE_PATH`` / ``N_SEEDS`` and run:

    uv run python mc_trajectories.py
"""

import os

import numpy as np
from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, extract_trajectory
from solver_defaults import build_default_config


FILE_PATH = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
             "src/results/arctic/fram-strait-fleet-week/output.pfg")

# Fewer seeds than the APE MC (25) -- the trajectory plot only needs enough
# ensemble members to render a visible spread. Overplot beyond ~10 seeds
# just muddles the figure.
MC_META_SEED = 0
N_SEEDS = 10
SEEDS = np.random.default_rng(MC_META_SEED).integers(
    2**31, size=N_SEEDS).tolist()

OUT_NAME = "mc_trajectories.npz"


def _positions(values, keys) -> np.ndarray:
    """(n_poses, 3) array of translations for the given key list."""
    traj = extract_trajectory(values, keys)
    return np.asarray(traj.positions_xyz, dtype=np.float64)


def main() -> None:
    print(f"Reading {FILE_PATH} ...")
    fg = read_from_pyfg_text(FILE_PATH)
    print(f"  {fg.num_poses} poses, {fg.num_landmarks} landmarks, "
          f"{len(fg.range_measurements)} range")

    chains = [c for c in fg.pose_variables if c]
    robot_chars = [c[0].name[0] for c in chains]
    print(f"Robots {robot_chars}, {len(SEEDS)} seeds")

    base_cfg = build_default_config(fg)

    # One-off pass to grab GT + odometry-only tracks (deterministic, no solve).
    warm = FactorGraphSolver(fg, base_cfg)
    gt_pos = {}
    odom_pos = {}
    n_poses_per_robot = {}
    for chain in chains:
        r = chain[0].name[0]
        keys = [warm.key_map[p.name] for p in chain]
        gt_pos[r] = _positions(warm.gt_values, keys)
        odom_pos[r] = _positions(warm.odom_values, keys)
        n_poses_per_robot[r] = len(chain)

    mc_pos: dict[str, list[np.ndarray]] = {r: [] for r in robot_chars}
    for i, seed in enumerate(SEEDS):
        print(f"\n--- seed {seed}  ({i + 1}/{len(SEEDS)}) ---")
        from copy import deepcopy
        cfg = deepcopy(base_cfg)
        cfg.seed = seed
        sol = FactorGraphSolver(fg, cfg)
        sol.solve()
        for chain in chains:
            r = chain[0].name[0]
            keys = [sol.key_map[p.name] for p in chain]
            mc_pos[r].append(_positions(sol.result, keys))
        print(f"  cached tracks for {len(robot_chars)} robots")

    save_kwargs: dict[str, np.ndarray] = {
        "seeds": np.asarray(SEEDS, dtype=np.int64),
    }
    for r in robot_chars:
        save_kwargs[f"gt_{r}"] = gt_pos[r]
        save_kwargs[f"odom_{r}"] = odom_pos[r]
        save_kwargs[f"mc_{r}"] = np.stack(mc_pos[r], axis=0)

    out_path = os.path.join(os.path.dirname(FILE_PATH), OUT_NAME)
    np.savez(out_path, **save_kwargs)
    print(f"\nSaved {out_path}  robots={robot_chars}  seeds={len(SEEDS)}")


if __name__ == "__main__":
    main()
