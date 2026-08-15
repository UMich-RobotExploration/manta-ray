#!/usr/bin/env python3
"""Paired per-pose APE Monte Carlo: straight-line vs refracted ranges.

For each seed, solve the factor graph twice: once with bellhop-measured
ranges and once with straight-line ranges, using the same root seed.
SeedSequence-spawning makes the odom and depth injection noise draws
identical across the two legs, and the range injection noise is identical
samples applied to different range sources. The remaining per-pose
difference isolates the mean-shift between refracted and straight-line ranges
(e.g. the refraction bias) from noise-realization variance.

Outputs (all PNGs, dpi=300, plus the .npz cache on the solve path):

  mc_<r>_paired_ape_delta.png    one paired-delta plot per robot
  mc_fleet_paired_ape_delta.png  fleet-wide paired delta (all robots stacked)
  mc_pooled_ape_dist.png         pooled APE violin (all robots x all seeds)
  mc_paired_ape.npz              per-robot delta + measured + straight-line arrays

`LOAD_NPZ` defaults to the canonical saved .npz next to the source .pfg, so
the script replots from cache by default. Set it to None to force a fresh
solve sweep.

Edit the constants block at the top, then `uv run python mc_paired_ape.py`.
"""

import os
from copy import deepcopy

import numpy as np
from py_factor_graph.io.pyfg_text import read_from_pyfg_text

from pyfg_to_gtsam import FactorGraphSolver, per_pose_ape
from solver_defaults import build_default_config
from visualize_solver import (plot_ape_distribution_compare,
                              plot_paired_ape_delta)

# FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-fleet-week/output.pfg"

# Draw the MC seed list deterministically from a meta-seed so reruns
# produce the same set of realizations without hand-picking seed values.
MC_META_SEED = 0
N_SEEDS = 25
SEEDS = np.random.default_rng(MC_META_SEED).integers(
    2**31, size=N_SEEDS).tolist()

ROBOT: str | None = None  # None = all robots; else robot char e.g. "A"
SAVE_DIR = os.path.dirname(FILE_PATH)
PREFIX = "mc"

# When set to a previously-saved mc_*_paired_ape.npz path, skip the solve
# sweep and replot from cache. Set to None to force a fresh solve sweep
# (necessary whenever SEEDS / N_SEEDS / MC_META_SEED change).
LOAD_NPZ: str | None = f"/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/{PREFIX}_paired_ape.npz"


def _plot_per_robot(deltas: dict[str, np.ndarray],
                    seeds: list[int] | np.ndarray) -> None:
    """Render one paired-delta plot per robot, with per-robot summary print."""
    seeds_list = list(seeds)
    for r, d in deltas.items():
        print(f"\nRobot {r} error summary (n_poses={d.shape[1]}):")
        print(f"  mean        = {d.mean():+.3f} m")
        print(f"  median      = {np.median(d):+.3f} m")
        print(f"  P(error>0)  = {(d > 0).mean():.3f}")
        plot_paired_ape_delta(d, seeds_list,
                              save_dir=SAVE_DIR,
                              prefix=f"{PREFIX}_{r}",
                              minuend_label="Straight-line",
                              subtrahend_label="Refracted",
                              robot_char=r,
                              show=False)


def _pool_apes(data) -> tuple[np.ndarray, np.ndarray]:
    """Return (pooled_measured, pooled_idealized) flat 1D arrays.

    Concatenates every `ape_measured_<r>` / `ape_idealized_<r>` array in
    `data` (works for both an np.load handle and the in-memory save_kwargs
    dict). Per-robot pose-count differences are flattened away; order is
    deterministic in insertion order of the source keys.
    """
    keys = data.files if hasattr(data, "files") else list(data.keys())
    m_chunks = [np.asarray(data[k]).ravel() for k in keys
                if k.startswith("ape_measured_")]
    t_chunks = [np.asarray(data[k]).ravel() for k in keys
                if k.startswith("ape_idealized_")]
    if not m_chunks or not t_chunks:
        raise RuntimeError(
            f"No ape_measured_*/ape_idealized_* arrays in cache (keys={keys})")
    return np.concatenate(m_chunks), np.concatenate(t_chunks)


def main() -> None:
    if LOAD_NPZ is not None:
        print(f"Loading {LOAD_NPZ} (skipping solves) ...")
        data = np.load(LOAD_NPZ)
        seeds = data["seeds"]
        deltas = {k[len("delta_"):]: data[k]
                  for k in data.files if k.startswith("delta_")}
        if not deltas:
            raise RuntimeError(
                f"No 'delta_*' arrays in {LOAD_NPZ}; "
                f"keys present: {list(data.files)}")
        print(f"Robots in cache: {list(deltas)}, seeds={list(seeds)}")
        pool_source = data
    else:
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

        deltas = {}
        save_kwargs: dict[str, np.ndarray] = {
            "seeds": np.asarray(SEEDS, dtype=np.int64),
        }
        for r in robot_chars:
            m = np.vstack(ape_measured[r])      # (n_seeds, n_poses_r)
            t = np.vstack(ape_idealized[r])     # (n_seeds, n_poses_r)
            d = t - m
            deltas[r] = d
            save_kwargs[f"delta_{r}"] = d
            save_kwargs[f"ape_measured_{r}"] = m
            save_kwargs[f"ape_idealized_{r}"] = t

        out_npz = os.path.join(SAVE_DIR, f"{PREFIX}_paired_ape.npz")
        np.savez(out_npz, **save_kwargs)
        print(f"\nSaved {out_npz}  robots={list(deltas)}")
        seeds = SEEDS
        pool_source = save_kwargs

    _plot_per_robot(deltas, seeds)

    # Fleet paired-delta: stack every robot's (n_seeds, n_poses) matrix into
    # one (n_robots * n_seeds, n_poses) matrix and feed to the same plotter.
    # Requires equal pose counts across robots; raise if not.
    pose_counts = {r: d.shape[1] for r, d in deltas.items()}
    if len(set(pose_counts.values())) != 1:
        raise RuntimeError(
            f"Fleet plot needs equal per-robot pose counts, got {pose_counts}")
    robots = list(deltas)
    fleet_delta = np.vstack([deltas[r] for r in robots])
    plot_paired_ape_delta(
        fleet_delta, list(seeds),
        save_dir=SAVE_DIR,
        prefix=f"{PREFIX}_fleet",
        minuend_label="Straight-line",
        subtrahend_label="Refracted",
        robot_char="all robots",
        show=False)
    print(f"\nFleet paired delta "
          f"({len(robots)} robots x {len(seeds)} seeds = "
          f"{fleet_delta.shape[0]} traces over {fleet_delta.shape[1]} poses): "
          f"mean={fleet_delta.mean():+.3f} m, "
          f"median={np.median(fleet_delta):+.3f} m, "
          f"P(>0)={(fleet_delta > 0).mean():.3f}")

    # Pooled APE violin: flatten every (seed, pose) sample across robots
    # for each condition and feed to the existing distribution plotter.
    m_pool, t_pool = _pool_apes(pool_source)
    pooled_path = os.path.join(SAVE_DIR, f"{PREFIX}_pooled_ape_dist.png")
    plot_ape_distribution_compare(
        ape_results=[m_pool, t_pool],
        labels=["Refracted Ranges", "Straight-line Ranges"],
        robot_char="all robots x all seeds",
        save_path=pooled_path)
    print(f"Pooled ATE (n={m_pool.size:,d} samples per condition): "
          f"refracted median={np.median(m_pool):.3f} m, "
          f"straight-line median={np.median(t_pool):.3f} m")
    print(f"Saved {pooled_path}")


if __name__ == "__main__":
    main()
