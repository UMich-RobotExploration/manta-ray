#!/usr/bin/env python3
"""Paper-quality overlay SSP profile comparison across environments.

Reads exported .npy SSP grids from `mantaray/data/<env>/ssp/` and renders
a single-panel figure with both regions overlaid: min/max envelope band
plus the mean profile per environment, on the same depth-and-speed axes
so the reader compares them directly.

Edit the ENV_DIRS and PLOT_LABELS constants and run:

    cd pymantaray/acoustics && uv run python plot_ssp_comparison.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt

DATA_ROOT = "/home/tko/repos/manta-ray/mantaray/data"
ENV_DIRS = ["beaufort_summer", "fram_strait_summer"]
PLOT_LABELS = ["Beaufort Sea", "Fram Strait"]
# Blue for the "reference" region, orange for the comparison.
PLOT_COLORS = ["#1f77b4", "#ff7f0e"]

OUT_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/ssp_comparison.png"

MAX_DEPTH_M = 500.0     # crop y-axis so the interesting near-surface structure dominates
BAND_ALPHA = 0.18       # low enough to let overlap read cleanly, high enough to see the band
MEAN_LW = 1.6


def load_ssp(env_dir: str):
    """Return (mean, min, max, depth) arrays computed across (lat, lon)."""
    ssp_dir = os.path.join(DATA_ROOT, env_dir, "ssp")
    x = np.load(os.path.join(ssp_dir, "x_coords.npy"))
    y = np.load(os.path.join(ssp_dir, "y_coords.npy"))
    z = np.load(os.path.join(ssp_dir, "depth_coords.npy"))
    nx, ny, nz = len(x), len(y), len(z)
    flat = np.load(os.path.join(ssp_dir, "ssp.npy"))
    if flat.size != nx * ny * nz:
        raise RuntimeError(
            f"{env_dir}: expected {nx*ny*nz} SSP values, got {flat.size}")
    # Grid3D layout: ix*ny*nz + iy*nz + iz  (per CLAUDE.md contract)
    ssp = flat.reshape(nx, ny, nz)
    ssp_2d = ssp.reshape(-1, nz)                # (n_profiles, n_depth)
    ssp_2d = np.where(np.isfinite(ssp_2d), ssp_2d, np.nan)
    mean = np.nanmean(ssp_2d, axis=0)
    smin = np.nanmin(ssp_2d, axis=0)
    smax = np.nanmax(ssp_2d, axis=0)
    return mean, smin, smax, z


def main() -> None:
    profiles = [load_ssp(d) for d in ENV_DIRS]

    # Compute x-limits from ONLY the visible-depth window so the axis fits
    # the top-500 m structure, not the full water column's speed range.
    def _cropped_bounds(mean, smin, smax, z):
        m = z <= MAX_DEPTH_M
        return np.nanmin(smin[m]), np.nanmax(smax[m])
    bounds = [_cropped_bounds(*p) for p in profiles]
    all_lo = min(lo for lo, _ in bounds)
    all_hi = max(hi for _, hi in bounds)
    pad = 0.02 * (all_hi - all_lo)
    xlim = (all_lo - pad, all_hi + pad)

    fig, ax = plt.subplots(figsize=(5.5, 4.4))
    for label, color, (mean, smin, smax, z) in zip(
            PLOT_LABELS, PLOT_COLORS, profiles):
        ax.fill_betweenx(z, smin, smax, color=color, alpha=BAND_ALPHA,
                         linewidth=0)
        ax.plot(mean, z, color=color, linewidth=MEAN_LW, label=label)

    ax.set_xlabel("Sound speed (m/s)", fontsize=11)
    ax.set_ylabel("Depth (m)", fontsize=11)
    ax.set_xlim(xlim)
    ax.set_ylim(MAX_DEPTH_M, 0.0)       # inverted, cropped to 500 m
    ax.grid(axis="both", which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.35)
    ax.grid(axis="both", which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.20)
    ax.minorticks_on()
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)

    ax.legend(frameon=True, fontsize=10, loc="lower left",
              handlelength=1.8, handletextpad=0.5, borderpad=0.4)

    fig.tight_layout()
    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    fig.savefig(OUT_PATH, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
