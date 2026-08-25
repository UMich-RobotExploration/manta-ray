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


DIVER_COLOR = "#1f77b4"


def main() -> None:
    agents, cfg = _load_fleet(CONFIG_PATH)

    divers = [a for a in agents if not a["is_surface"]]
    surface = [a for a in agents if a["is_surface"]]
    if not divers:
        raise RuntimeError("No diver robots detected; nothing to plot.")

    d_min = min(a["target_depth"] for a in divers)
    d_max = max(a["target_depth"] for a in divers)

    fig, ax_xy = plt.subplots(figsize=(5.6, 5.0), constrained_layout=True)

    # XY footprint in kilometres. Depth information is deferred to the
    # accompanying table (all divers share a single marker colour here).
    for a in divers:
        ax_xy.scatter([a["x"] / 1000.0], [a["y"] / 1000.0], s=200,
                      c=DIVER_COLOR, marker="o",
                      edgecolors="black", linewidths=1.0, zorder=3)
    for a in surface:
        ax_xy.scatter([a["x"] / 1000.0], [a["y"] / 1000.0], s=230,
                      c=SURFACE_COLOR, marker="s",
                      edgecolors="black", linewidths=1.2, zorder=4)
    for a in agents:
        ax_xy.annotate(a["label"],
                       (a["x"] / 1000.0, a["y"] / 1000.0),
                       textcoords="offset points", xytext=(0, 14),
                       ha="center", fontsize=13, fontweight="bold")

    ax_xy.set_xlabel("x (km)", fontsize=15)
    ax_xy.set_ylabel("y (km)", fontsize=15)
    ax_xy.set_aspect("equal", adjustable="datalim")
    ax_xy.grid(True, which="major", linestyle="-", color="#333333",
               linewidth=0.9, alpha=0.35)
    ax_xy.grid(True, which="minor", linestyle="-", color="#888888",
               linewidth=0.4, alpha=0.20)
    ax_xy.minorticks_on()
    ax_xy.set_axisbelow(True)
    for spine in ("top", "right"):
        ax_xy.spines[spine].set_visible(False)
    ax_xy.tick_params(axis="both", which="both", length=3, labelsize=13)

    surface_legend = Line2D([0], [0], marker="s", color="w",
                             markerfacecolor=SURFACE_COLOR,
                             markeredgecolor="black", markeredgewidth=1.2,
                             markersize=11, label="Fixed float")
    diver_legend = Line2D([0], [0], marker="o", color="w",
                           markerfacecolor=DIVER_COLOR,
                           markeredgecolor="black", markeredgewidth=1.0,
                           markersize=11, label="Diver")
    ax_xy.legend(handles=[surface_legend, diver_legend],
                 loc="lower center", bbox_to_anchor=(0.5, 1.02),
                 frameon=True, fontsize=12, borderpad=0.4,
                 handletextpad=0.5, ncol=2)

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    fig.savefig(OUT_PATH, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"Wrote {OUT_PATH}")
    print(f"  agents: {len(agents)} total "
          f"({len(surface)} surface, {len(divers)} divers, "
          f"target depths {d_min:.0f}..{d_max:.0f} m)")


if __name__ == "__main__":
    main()
