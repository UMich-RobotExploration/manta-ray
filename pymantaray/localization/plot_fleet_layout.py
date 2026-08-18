#!/usr/bin/env python3
"""Paper-quality fleet-layout figure driven by a sim-config JSON.

Two panels sharing a viridis colormap on target depth:
  * left  — top-down XY footprint of every agent (surface floats and divers)
  * right — depth ladder: one column per agent, bar from surface to
            target depth, marker at the target

Reads the sim config JSON directly so the figure stays in sync with any
future config edits. Surface floats are auto-detected as any robot whose
``start_offset_seconds`` exceeds the mission duration (that value keeps
them locked in ``kHoldSurface`` for the full run, per
``CurrentDriftRobot.cpp``).

Edit the constants at the top and run:

    uv run python plot_fleet_layout.py
"""

import json
import os

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.lines import Line2D


CONFIG_PATH = ("/home/tko/repos/manta-ray/mantaray/sim_config/"
               "beaufort_fleet_week_sim.json")
OUT_PATH = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
            "src/results/arctic/beaufort-fleet-week/fleet_layout.png")

# Robot index -> pfg letter. 'L' is reserved for landmarks in PyFactorGraph
# (see PfgWriter.cpp), so index 11 maps to 'M'. This mirrors robotName().
_ROBOT_ALPHABET = "ABCDEFGHIJKMNOPQRSTUVWXYZ"

SURFACE_COLOR = "#e6a23c"   # amber, distinct from any viridis stop
DEPTH_CMAP = plt.cm.viridis_r


def _load_fleet(config_path: str):
    """Parse the sim JSON into a list of agent dicts + return the raw cfg.

    A robot is treated as a surface beacon when either
      * its ``type`` is ``constant_vel`` (station-keeping / beacon), or
      * its ``type`` is ``current_drift`` and ``start_offset_seconds`` exceeds
        the mission duration (locked in ``kHoldSurface`` for the whole run).
    """
    with open(config_path) as f:
        cfg = json.load(f)

    duration_s = float(cfg["timing"]["end_time_hours"]) * 3600.0
    agents = []
    for i, r in enumerate(cfg["robots"]):
        rtype = r.get("type", "current_drift")
        offset = float(r.get("start_offset_seconds", 0.0))
        pinned_current_drift = (rtype == "current_drift"
                                and offset > duration_s)
        is_surface = rtype == "constant_vel" or pinned_current_drift
        agents.append({
            "idx": i,
            "label": _ROBOT_ALPHABET[i],
            "type": rtype,
            "x": float(r["position"][0]),
            "y": float(r["position"][1]),
            "z0": float(r["position"][2]),
            "target_depth": float(r.get("target_depth", 0.0)),
            "start_offset_s": offset,
            "is_surface": is_surface,
        })
    return agents, cfg


def main() -> None:
    agents, cfg = _load_fleet(CONFIG_PATH)

    divers = [a for a in agents if not a["is_surface"]]
    surface = [a for a in agents if a["is_surface"]]
    if not divers:
        raise RuntimeError("No diver robots detected; nothing to color.")

    d_min = min(a["target_depth"] for a in divers)
    d_max = max(a["target_depth"] for a in divers)
    norm = plt.Normalize(vmin=d_min, vmax=d_max)

    def _diver_color(depth: float):
        return DEPTH_CMAP(norm(depth))

    fig, (ax_xy, ax_z) = plt.subplots(
        1, 2, figsize=(9.5, 4.4),
        gridspec_kw={"width_ratios": [1.15, 1.0]},
        constrained_layout=True,
    )

    # -------- LEFT: XY footprint --------
    for a in divers:
        ax_xy.scatter([a["x"]], [a["y"]], s=200,
                      c=[_diver_color(a["target_depth"])],
                      marker="o", edgecolors="black", linewidths=1.0, zorder=3)
    for a in surface:
        ax_xy.scatter([a["x"]], [a["y"]], s=230, c=SURFACE_COLOR,
                      marker="s", edgecolors="black", linewidths=1.2, zorder=4)
    for a in agents:
        ax_xy.annotate(a["label"], (a["x"], a["y"]),
                       textcoords="offset points", xytext=(0, 13),
                       ha="center", fontsize=9, fontweight="bold")

    ax_xy.set_xlabel("x (m)", fontsize=11)
    ax_xy.set_ylabel("y (m)", fontsize=11)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, which="major", linestyle="-", color="#333333",
               linewidth=0.9, alpha=0.35)
    ax_xy.grid(True, which="minor", linestyle="-", color="#888888",
               linewidth=0.4, alpha=0.20)
    ax_xy.minorticks_on()
    ax_xy.set_axisbelow(True)
    for spine in ("top", "right"):
        ax_xy.spines[spine].set_visible(False)
    ax_xy.tick_params(axis="both", which="both", length=3)

    surface_legend = Line2D([0], [0], marker="s", color="w",
                             markerfacecolor=SURFACE_COLOR,
                             markeredgecolor="black", markeredgewidth=1.2,
                             markersize=10, label="Surface float")
    diver_legend = Line2D([0], [0], marker="o", color="w",
                           markerfacecolor="0.55", markeredgecolor="black",
                           markeredgewidth=1.0, markersize=10, label="Diver")
    ax_xy.legend(handles=[surface_legend, diver_legend], loc="upper right",
                 frameon=True, fontsize=9, borderpad=0.4,
                 handletextpad=0.5)

    # -------- RIGHT: depth ladder --------
    ordered_divers = sorted(divers, key=lambda a: a["target_depth"])
    columns = surface + ordered_divers
    positions = np.arange(len(columns))

    for pos, a in zip(positions, columns):
        if a["is_surface"]:
            ax_z.scatter([pos], [0], s=230, c=SURFACE_COLOR, marker="s",
                         edgecolors="black", linewidths=1.2, zorder=4)
        else:
            color = _diver_color(a["target_depth"])
            ax_z.plot([pos, pos], [0, a["target_depth"]], color=color,
                      linewidth=7, solid_capstyle="butt", alpha=0.85, zorder=2)
            ax_z.scatter([pos], [a["target_depth"]], s=200, c=[color],
                         marker="o", edgecolors="black", linewidths=1.0,
                         zorder=3)

    ax_z.set_xticks(positions)
    ax_z.set_xticklabels([a["label"] for a in columns], fontsize=9)
    ax_z.set_xlabel("Agent", fontsize=11)
    ax_z.set_ylabel("Target depth (m)", fontsize=11)
    ax_z.set_ylim(d_max * 1.08, -0.06 * d_max)  # inverted with a little sky
    ax_z.grid(axis="y", which="major", linestyle="-", color="#333333",
              linewidth=0.9, alpha=0.35)
    ax_z.grid(axis="y", which="minor", linestyle="-", color="#888888",
              linewidth=0.4, alpha=0.20)
    ax_z.minorticks_on()
    ax_z.set_axisbelow(True)
    for spine in ("top", "right"):
        ax_z.spines[spine].set_visible(False)
    ax_z.tick_params(axis="both", which="both", length=3)

    # Zero-line marker: solid horizontal at the surface so the sea-level
    # datum reads at a glance.
    ax_z.axhline(0.0, color="#888888", linewidth=0.8, zorder=1)

    # Shared colorbar spans both axes.
    sm = ScalarMappable(cmap=DEPTH_CMAP, norm=norm)
    sm.set_array([])
    cbar = fig.colorbar(sm, ax=[ax_xy, ax_z],
                        label="Diver target depth (m)",
                        pad=0.02, fraction=0.045, shrink=0.9)
    cbar.ax.tick_params(length=3)

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    fig.savefig(OUT_PATH, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {OUT_PATH}")
    print(f"  agents: {len(agents)} total "
          f"({len(surface)} surface, {len(divers)} divers, "
          f"target depths {d_min:.0f}..{d_max:.0f} m)")


if __name__ == "__main__":
    main()
