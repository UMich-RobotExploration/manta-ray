#!/usr/bin/env python3
"""Paper-quality MC trajectory figure.

Two panels sharing per-robot color:

  * Panel A -- top-down XY footprint of the fleet. Ground truth is drawn
    as a bold line per robot; each MC seed's solved trajectory as a
    thin translucent line. Highlights horizontal drift and the spread
    of the estimate across seeds.

  * Panel B -- axonometric (oblique-3D) view revealing depth. The full
    3D track (x, y, -z) is projected into 2D via a fixed viewpoint,
    which "rotates" the x-z plane through the y axis so the diving
    cycles of each agent become visible without hiding horizontal
    context. Same per-robot color scheme as Panel A.

Reads the cache produced by ``mc_trajectories.py`` (no re-solving here)
and writes ``mc_trajectories.png`` next to it. Edit ``NPZ_PATH`` and:

    uv run python plot_mc_trajectories.py
"""

import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


NPZ_PATH = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
            "src/results/arctic/fram-strait-fleet-week/mc_trajectories.npz")
OUT_PATH = os.path.join(os.path.dirname(NPZ_PATH), "mc_trajectories.png")

# Draw first N seeds -- enough to show ensemble spread without a wall of ink.
MAX_SEEDS_PLOTTED = 10

# Rotated x-z plane about the vertical axis: the viewing plane is
# aligned with the AZ_DEG azimuth so both x and y contribute to the
# horizontal axis, and the vertical axis is depth in metres. This
# literally is "the x-z plane, rotated" -- no unit-mixing tilt.
AZ_DEG = 35.0

# Alpha budget per MC seed. Ensemble is on top of GT but rendered as a
# soft translucent halo so it never obscures the mean trace.
MC_ALPHA = 0.45
MC_LW = 0.7
GT_LW = 1.8


def _project_side(positions: np.ndarray,
                  az_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Side view along an azimuth ``az_deg`` (rotation about vertical).

    Returns (u_km, v_m):
      u = ( x cos(az) + y sin(az) ) / 1000  -- rotated horizontal in km
      v = -z                                -- depth in m, deeper = lower

    Equivalent to viewing the fleet from a horizontal camera whose bearing
    is ``az_deg`` east of the +x axis. Both x and y are visible in the
    horizontal because they project onto the viewing plane.
    """
    az = np.radians(az_deg)
    x = positions[:, 0]
    y = positions[:, 1]
    z = positions[:, 2]
    u = (x * np.cos(az) + y * np.sin(az)) / 1000.0
    v = -z
    return u, v


def _load(npz_path: str):
    d = np.load(npz_path)
    seeds = d["seeds"]
    robots = sorted({k.split("_", 1)[1] for k in d.files
                     if k.startswith("gt_")})
    return d, seeds, robots


def _robot_palette(robots: list[str]) -> dict[str, tuple]:
    """One distinct colour per robot. tab20 gives 20 slots -- enough for 11."""
    cmap = plt.get_cmap("tab20")
    return {r: cmap(i * 2) for i, r in enumerate(robots)}


def _draw_panel_xy(ax, data, robots, palette, n_seeds_plot) -> None:
    for r in robots:
        color = palette[r]
        gt = data[f"gt_{r}"] / 1000.0
        mc = data[f"mc_{r}"] / 1000.0

        # MC ensemble
        for k in range(min(n_seeds_plot, mc.shape[0])):
            ax.plot(mc[k, :, 0], mc[k, :, 1],
                    color=color, alpha=MC_ALPHA, linewidth=MC_LW,
                    solid_capstyle="round", zorder=2)
        # GT bold on top
        ax.plot(gt[:, 0], gt[:, 1],
                color=color, linewidth=GT_LW, zorder=4)
        # Start marker
        ax.scatter([gt[0, 0]], [gt[0, 1]], s=32,
                   facecolor=color, edgecolor="black", linewidths=0.7,
                   zorder=5)
        # Robot label at the trajectory start
        ax.annotate(r, (gt[0, 0], gt[0, 1]),
                    textcoords="offset points", xytext=(6, 6),
                    fontsize=8.5, fontweight="bold", color="black",
                    zorder=6)

    ax.set_xlabel("x (km)", fontsize=11)
    ax.set_ylabel("y (km)", fontsize=11)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.30)
    ax.grid(True, which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.20)
    ax.minorticks_on()
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)
    ax.set_title("(a) Top-down (X-Y)", fontsize=11, pad=6)


def _draw_panel_iso(ax, data, robots, palette, n_seeds_plot) -> None:
    for r in robots:
        color = palette[r]
        gt = data[f"gt_{r}"]
        mc = data[f"mc_{r}"]

        for k in range(min(n_seeds_plot, mc.shape[0])):
            u, v = _project_side(mc[k], AZ_DEG)
            ax.plot(u, v,
                    color=color, alpha=MC_ALPHA, linewidth=MC_LW,
                    solid_capstyle="round", zorder=2)
        u_gt, v_gt = _project_side(gt, AZ_DEG)
        ax.plot(u_gt, v_gt,
                color=color, linewidth=GT_LW, zorder=4)
        # Anchor the marker + label at the deepest point of the trajectory
        # so labels spread vertically by target depth instead of colliding
        # against the surface. Fixed float (A) sits at v ~= 0 and lands
        # against the surface line -- that reads correctly.
        idx_deep = int(np.argmin(v_gt))
        ax.scatter([u_gt[idx_deep]], [v_gt[idx_deep]], s=32,
                   facecolor=color, edgecolor="black", linewidths=0.7,
                   zorder=5)
        ax.annotate(r, (u_gt[idx_deep], v_gt[idx_deep]),
                    textcoords="offset points", xytext=(6, -10),
                    fontsize=8.5, fontweight="bold", color="black",
                    zorder=6)

    ax.set_xlabel(f"horizontal along bearing {AZ_DEG:.0f}° (km)",
                  fontsize=11)
    ax.set_ylabel("depth-negated z (m)", fontsize=11)
    ax.grid(True, which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.30)
    ax.grid(True, which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.20)
    ax.minorticks_on()
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)
    ax.set_title(f"(b) Rotated side view (bearing {AZ_DEG:.0f}°)",
                 fontsize=11, pad=6)


def _draw_legend(fig, robots, palette, n_seeds_plot) -> None:
    swatches = [
        Line2D([0], [0], color=palette[r], linewidth=GT_LW, label=r)
        for r in robots
    ]
    # Also a "seed" swatch showing what an MC line looks like.
    ensemble = Line2D([0], [0], color="0.35", linewidth=MC_LW,
                      alpha=0.85,
                      label=f"MC seed ({n_seeds_plot} shown)")
    handles = swatches + [ensemble]
    fig.legend(handles=handles,
               loc="lower center", bbox_to_anchor=(0.5, -0.09),
               ncol=min(len(handles), 12),
               frameon=True, fontsize=9,
               handlelength=1.5, handletextpad=0.4,
               columnspacing=1.1, borderpad=0.4)


def main() -> None:
    if not os.path.exists(NPZ_PATH):
        raise FileNotFoundError(
            f"MC trajectory cache missing: {NPZ_PATH}\n"
            f"Run mc_trajectories.py first to produce it.")

    print(f"Loading {NPZ_PATH} ...")
    data, seeds, robots = _load(NPZ_PATH)
    n_seeds_plot = min(MAX_SEEDS_PLOTTED, seeds.size)
    print(f"Robots: {robots}  (plotting {n_seeds_plot} of {seeds.size} seeds)")

    palette = _robot_palette(robots)

    fig, (ax_xy, ax_iso) = plt.subplots(
        1, 2, figsize=(11.4, 5.2), constrained_layout=True,
        gridspec_kw={"width_ratios": [1.0, 1.15]})

    _draw_panel_xy(ax_xy, data, robots, palette, n_seeds_plot)
    _draw_panel_iso(ax_iso, data, robots, palette, n_seeds_plot)
    _draw_legend(fig, robots, palette, n_seeds_plot)

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    # Leave room at the bottom for the legend that lives below the axes.
    fig.savefig(OUT_PATH, dpi=400, bbox_inches="tight",
                pad_inches=0.15)
    plt.close(fig)
    print(f"Wrote {OUT_PATH}")


if __name__ == "__main__":
    main()
