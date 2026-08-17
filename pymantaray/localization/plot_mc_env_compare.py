#!/usr/bin/env python3
"""Cross-environment fleet-pooled translation RMSE.

Reads two or more paired-MC caches (one per environment) and renders a
grouped bar chart: one group per environment, two bars per group
(straight-line vs refracted). Intended as a paper figure that shows
directly how refraction bias affects estimator RMSE across sound-speed
regimes.

Edit the ``ENV_CACHES`` and ``OUT_PATH`` constants and run:

    uv run python plot_mc_env_compare.py

Each cache is produced by ``mc_paired_ape.py`` (single environment) and
consumed here for cross-environment comparison.
"""

import os

import numpy as np
import matplotlib.pyplot as plt


# One tuple per environment: (display_label, path/to/mc_paired_ape.npz).
ENV_CACHES = [
    ("Beaufort Sea",
     "/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
     "src/results/arctic/beaufort-fleet-week/mc_paired_ape.npz"),
    ("Fram Strait",
     "/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
     "src/results/arctic/fram-strait-fleet-week/mc_paired_ape.npz"),
]

OUT_PATH = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
            "src/results/arctic/mc_env_compare_rmse.png")


def _fleet_pooled_rmse(npz_path: str) -> tuple[float, float, int]:
    """Return (rmse_straight_line, rmse_refracted, n_samples_per_condition).

    ``ape_measured_<r>`` = REFRACTED solve, ``ape_idealized_<r>`` =
    STRAIGHT-LINE solve. Pools every (seed, pose) sample across all robots
    into one flat array per condition and reports RMSE.
    """
    data = np.load(npz_path)
    keys = list(data.files)
    refr = np.concatenate([np.asarray(data[k]).ravel() for k in keys
                           if k.startswith("ape_measured_")])
    line = np.concatenate([np.asarray(data[k]).ravel() for k in keys
                           if k.startswith("ape_idealized_")])
    if refr.size != line.size:
        raise RuntimeError(
            f"{npz_path}: paired conditions have unequal sample counts "
            f"({refr.size} refracted vs {line.size} straight-line). "
            f"Cache is corrupt or out of sync.")
    return (float(np.sqrt(np.mean(line ** 2))),
            float(np.sqrt(np.mean(refr ** 2))),
            int(line.size))


def main() -> None:
    labels: list[str] = []
    rmse_line: list[float] = []
    rmse_refr: list[float] = []
    for name, path in ENV_CACHES:
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"MC cache missing for {name}: {path}\n"
                f"Run mc_paired_ape.py against that environment first.")
        line, refr, n = _fleet_pooled_rmse(path)
        labels.append(name)
        rmse_line.append(line)
        rmse_refr.append(refr)
        print(f"  {name:>15s}: n={n:>8,d}  "
              f"straight-line RMSE = {line:.3f} m, "
              f"refracted RMSE = {refr:.3f} m")

    x = np.arange(len(labels))
    width = 0.36

    fig, ax = plt.subplots(figsize=(5.6, 3.6))
    ax.bar(x - width / 2, rmse_line, width,
           label="Straight-line ranges",
           color="#1f77b4", edgecolor="none")
    ax.bar(x + width / 2, rmse_refr, width,
           label="Refracted (bellhop) ranges",
           color="#d62728", edgecolor="none")

    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, fontsize=11)
    ax.set_ylabel("Translation RMSE (m)", fontsize=11)
    ax.grid(axis="y", which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.55)
    ax.grid(axis="y", which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.25)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)

    ax.legend(frameon=True, fontsize=10, loc="upper center",
              bbox_to_anchor=(0.5, 1.02), ncol=2,
              handlelength=1.4, handletextpad=0.5, columnspacing=1.2,
              borderpad=0.4)

    ymax = max(max(rmse_line), max(rmse_refr))
    ymin = min(min(rmse_line), min(rmse_refr))
    ax.set_ylim(ymin * 0.5, ymax * 3.5)

    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    fig.savefig(OUT_PATH, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
