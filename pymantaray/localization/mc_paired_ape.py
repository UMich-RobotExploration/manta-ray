#!/usr/bin/env python3
"""Paired-Monte-Carlo runner: solve the factor graph twice per seed
(bellhop-measured vs straight-line ranges) and write a per-robot APE
cache to ``mc_paired_ape.npz`` next to the source pfg.

Both conditions use identical odom + depth noise draws (via
SeedSequence spawning inside FactorGraphSolver) and identical
range-noise samples applied to different range sources, so the
per-pose delta between the two solves isolates the refraction-bias
effect from noise-realization variance.

Plotting lives in a separate module (`plot_mc_paired_ape.py`) so the
runner has one job. To rebuild the cache:

    uv run python mc_paired_ape.py

Then plot the results with:

    uv run python plot_mc_paired_ape.py
"""

import os
from copy import deepcopy

import numpy as np
from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, per_pose_ape
from solver_defaults import build_default_config


# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-fleet-week/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/fram-strait-fleet-week/output.pfg"

# Draw the MC seed list deterministically from a meta-seed so reruns
# produce the same set of realizations without hand-picking seed values.
MC_META_SEED = 0
N_SEEDS = 25
SEEDS = np.random.default_rng(MC_META_SEED).integers(
    2**31, size=N_SEEDS).tolist()

ROBOT: str | None = None  # None = all robots; else robot char e.g. "A"
SAVE_DIR = os.path.dirname(FILE_PATH)
PREFIX = "mc"


def main() -> None:
    print(f"Reading {FILE_PATH} ...")
    fg_data = read_from_pyfg_text(FILE_PATH)
    print(f"  {fg_data.num_poses} poses, "
          f"{fg_data.num_landmarks} landmarks, "
          f"{len(fg_data.range_measurements)} range")

    chains = [c for c in fg_data.pose_variables if c]
    if not chains:
        raise RuntimeError("No pose chains in factor graph")
    if ROBOT is None:
        robot_chars = [c[0].name[0] for c in chains]
    else:
        robot_chars = [ROBOT]
    print(f"Scoring robots {robot_chars} across {len(SEEDS)} seeds")

    # Build the base config once. Variants are deepcopy + field-mutation
    # so the diff between the two legs is explicit and the builder is not
    # re-invoked mid-sweep (avoids drift if defaults are edited live).
    base_config = build_default_config(fg_data)

    ape_measured: dict[str, list[np.ndarray]] = {r: [] for r in robot_chars}
    ape_idealized: dict[str, list[np.ndarray]] = {r: [] for r in robot_chars}

    for i, seed in enumerate(SEEDS):
        print(f"\n--- seed {seed}  ({i + 1}/{len(SEEDS)}) ---")

        cfg_m = deepcopy(base_config)
        cfg_m.seed = seed

        cfg_t = deepcopy(base_config)
        cfg_t.seed = seed
        cfg_t.use_true_ranges = True

        sol_m = FactorGraphSolver(fg_data, cfg_m)
        sol_m.solve()
        sol_t = FactorGraphSolver(fg_data, cfg_t)
        sol_t.solve()

        ape_m_dict = per_pose_ape(sol_m)
        ape_t_dict = per_pose_ape(sol_t)
        for r in robot_chars:
            if r not in ape_m_dict:
                raise KeyError(
                    f"Robot {r!r} not in factor graph "
                    f"(available: {sorted(ape_m_dict)})")
            ape_m = ape_m_dict[r]
            ape_t = ape_t_dict[r]
            if ape_m.shape != ape_t.shape:
                raise RuntimeError(
                    f"per-pose APE shape mismatch at seed {seed}, "
                    f"robot {r}: measured={ape_m.shape} "
                    f"idealized={ape_t.shape}")
            ape_measured[r].append(ape_m)
            ape_idealized[r].append(ape_t)
            print(f"  [{r}] measured mean={ape_m.mean():.3f} m, "
                  f"idealized mean={ape_t.mean():.3f} m")

    # Stack per-seed arrays into (n_seeds, n_poses) matrices per robot and
    # persist under stable key names. `ape_measured_<r>` holds the refracted
    # (bellhop-measured) APE; `ape_idealized_<r>` holds the straight-line
    # APE; `delta_<r>` is idealized minus measured.
    save_kwargs: dict[str, np.ndarray] = {
        "seeds": np.asarray(SEEDS, dtype=np.int64),
    }
    for r in robot_chars:
        m = np.vstack(ape_measured[r])
        t = np.vstack(ape_idealized[r])
        save_kwargs[f"delta_{r}"] = t - m
        save_kwargs[f"ape_measured_{r}"] = m
        save_kwargs[f"ape_idealized_{r}"] = t

    out_npz = os.path.join(SAVE_DIR, f"{PREFIX}_paired_ape.npz")
    np.savez(out_npz, **save_kwargs)
    print(f"\nSaved {out_npz}  robots={robot_chars}  seeds={len(SEEDS)}")


if __name__ == "__main__":
    main()
