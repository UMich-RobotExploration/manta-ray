#!/usr/bin/env python3
"""Plotter for the paired-MC APE cache produced by ``mc_paired_ape.py``.

Reads a single ``mc_paired_ape.npz`` and renders four PNGs alongside it:

  mc_<r>_paired_ape_delta.png     one paired-delta plot per robot
  mc_fleet_paired_ape_delta.png   fleet-wide paired delta (all robots stacked)
  mc_pooled_ape_dist.png          pooled APE violin (refracted vs straight-line)
  mc_translation_rmse_bar.png     per-robot translation RMSE bar

Edit ``NPZ_PATH`` and run:

    uv run python plot_mc_paired_ape.py

Cross-environment comparison lives in ``plot_mc_env_compare.py``.
"""

import os

import numpy as np
import matplotlib.pyplot as plt

from visualize_solver import (plot_ape_distribution_compare,
                              plot_paired_ape_delta)


NPZ_PATH = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
            "src/results/arctic/fram-strait-fleet-week/mc_paired_ape.npz")
SAVE_DIR = os.path.dirname(NPZ_PATH)
PREFIX = "mc"


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
    """Return (pooled_measured, pooled_idealized) flat 1D arrays."""
    keys = data.files if hasattr(data, "files") else list(data.keys())
    m_chunks = [np.asarray(data[k]).ravel() for k in keys
                if k.startswith("ape_measured_")]
    t_chunks = [np.asarray(data[k]).ravel() for k in keys
                if k.startswith("ape_idealized_")]
    if not m_chunks or not t_chunks:
        raise RuntimeError(
            f"No ape_measured_*/ape_idealized_* arrays in cache (keys={keys})")
    return np.concatenate(m_chunks), np.concatenate(t_chunks)


def _plot_pooled_ape_box(data, save_path: str) -> None:
    """Paper-clean boxplot of pooled APE: refracted vs straight-line.

    Two boxes side-by-side. No KDE curves, no outlier fliers -- clean
    median / IQR / 1.5*IQR whiskers only. Log y-axis so the several-order
    APE spread reads cleanly. Mean shown as a white triangle so the
    reader can see how far mean sits from median (indicative of skew).
    """
    refr, line = _pool_apes(data)   # measured = refracted, idealized = straight-line
    fig, ax = plt.subplots(figsize=(4.4, 3.6))
    bp = ax.boxplot(
        [line, refr],
        positions=[0, 1],
        widths=0.55,
        patch_artist=True,
        showfliers=False,
        showmeans=True,
        medianprops=dict(color="black", linewidth=1.4),
        whiskerprops=dict(color="black", linewidth=0.9),
        capprops=dict(color="black", linewidth=0.9),
        boxprops=dict(linewidth=0.6),
        meanprops=dict(marker="^", markerfacecolor="white",
                       markeredgecolor="black", markersize=6,
                       markeredgewidth=0.9),
    )
    for patch, c in zip(bp["boxes"], ["#1f77b4", "#d62728"]):
        patch.set_facecolor(c)
        patch.set_edgecolor("none")

    ax.set_xticks([0, 1])
    ax.set_xticklabels(["Straight-line\nranges", "Refracted\nranges"],
                       fontsize=10)
    ax.set_ylabel("Translation APE (m)", fontsize=11)
    ax.set_yscale("log")
    ax.grid(axis="y", which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.55)
    ax.grid(axis="y", which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.25)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)
    ax.tick_params(axis="x", length=0)

    fig.tight_layout()
    fig.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def _plot_translation_rmse_bar(data, robots: list[str],
                                save_path: str) -> None:
    """Per-robot translation RMSE, two bars per robot.

    ``ape_measured_<r>`` in the cache is the REFRACTED (bellhop-measured)
    solve; ``ape_idealized_<r>`` is the STRAIGHT-LINE solve. Bar labels
    reflect the underlying condition, not the storage key names.
    """
    labels = sorted(robots)
    rmse_refr = np.array([
        np.sqrt(np.mean(np.asarray(data[f"ape_measured_{r}"]) ** 2))
        for r in labels])
    rmse_line = np.array([
        np.sqrt(np.mean(np.asarray(data[f"ape_idealized_{r}"]) ** 2))
        for r in labels])

    x = np.arange(len(labels))
    width = 0.4

    fig, ax = plt.subplots(figsize=(max(6.4, 0.75 * len(labels)), 3.6))
    ax.bar(x - width / 2, rmse_line, width,
           label="Straight-line ranges",
           color="#1f77b4", edgecolor="none")
    ax.bar(x + width / 2, rmse_refr, width,
           label="Refracted (bellhop) ranges",
           color="#d62728", edgecolor="none")

    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=10)
    ax.set_ylabel("Translation RMSE (m)", fontsize=11)
    ax.grid(axis="y", which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.55)
    ax.grid(axis="y", which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.25)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)

    ax.legend(frameon=True, fontsize=9, loc="upper center",
              bbox_to_anchor=(0.5, 1.02), ncol=2,
              handlelength=1.4, handletextpad=0.5, columnspacing=1.2,
              borderpad=0.4)

    ymax = max(rmse_line.max(), rmse_refr.max())
    ymin = min(rmse_line.min(), rmse_refr.min())
    ax.set_ylim(ymin * 0.5, ymax * 3.5)

    fig.tight_layout()
    fig.savefig(save_path, dpi=300, bbox_inches="tight")
    plt.close(fig)


def main() -> None:
    if not os.path.exists(NPZ_PATH):
        raise FileNotFoundError(
            f"MC cache missing: {NPZ_PATH}\n"
            f"Run mc_paired_ape.py first to produce the cache.")

    print(f"Loading {NPZ_PATH} ...")
    data = np.load(NPZ_PATH)
    seeds = data["seeds"]
    deltas = {k[len("delta_"):]: data[k]
              for k in data.files if k.startswith("delta_")}
    if not deltas:
        raise RuntimeError(
            f"No 'delta_*' arrays in {NPZ_PATH}; "
            f"keys present: {list(data.files)}")
    robots = list(deltas)
    print(f"Robots in cache: {robots}, seeds={list(seeds)}")

    _plot_per_robot(deltas, seeds)

    # Fleet paired-delta: stack every robot's (n_seeds, n_poses) matrix into
    # one (n_robots * n_seeds, n_poses) matrix and feed to the same plotter.
    # Requires equal pose counts across robots; raise if not.
    pose_counts = {r: d.shape[1] for r, d in deltas.items()}
    if len(set(pose_counts.values())) != 1:
        raise RuntimeError(
            f"Fleet plot needs equal per-robot pose counts, got {pose_counts}")
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

    m_pool, t_pool = _pool_apes(data)
    pooled_path = os.path.join(SAVE_DIR, f"{PREFIX}_pooled_ape_dist.png")
    plot_ape_distribution_compare(
        ape_results=[m_pool, t_pool],
        labels=["Refracted Ranges", "Straight-line Ranges"],
        robot_char="all robots x all seeds",
        save_path=pooled_path,
        title="All Agents Across Monte Carlo")
    print(f"Pooled ATE (n={m_pool.size:,d} samples per condition): "
          f"refracted median={np.median(m_pool):.3f} m, "
          f"straight-line median={np.median(t_pool):.3f} m")
    print(f"Saved {pooled_path}")

    rmse_bar_path = os.path.join(SAVE_DIR, f"{PREFIX}_translation_rmse_bar.png")
    _plot_translation_rmse_bar(data, robots, rmse_bar_path)
    print(f"Saved {rmse_bar_path}")


if __name__ == "__main__":
    main()
