#!/usr/bin/env python3
"""
Script that helps visualize and debug range data.
"""

import os

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# Import parsing function
from py_factor_graph.io.pyfg_text import read_from_pyfg_text
from py_factor_graph.utils.plot_utils import (
    draw_pose_3d,
    draw_landmark_variable_3d,
    draw_traj_3d,
    draw_range_measurement_3d,
)
from py_factor_graph.variables import PoseVariable3D
from py_factor_graph.modifiers import make_all_ranges_perfect

from vtk_plots import plot_range_errors_vtk


FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-no-multi/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-simple/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/lbl-float/output.pfg"
# FILE_PATH = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl/output.pfg"
FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
WORK_DIR = os.path.dirname(FILE_PATH)


def plot_range_histogram(fg_data, save_dir: str | None = None):
    """Plot a histogram comparing measured range distances vs true distances.

    Args:
        fg_data: FactorGraphData object with range measurements
        save_dir: Directory to save figure. If None, only shows interactively.
    """
    if not fg_data.range_measurements:
        print("No range measurements to compare.")
        return

    true_fg = make_all_ranges_perfect(fg_data)

    measured_dists = [m.dist for m in fg_data.range_measurements]
    true_dists = [m.dist for m in true_fg.range_measurements]
    errors = [m - t for m, t in zip(measured_dists, true_dists)]
    pct_errors = [100.0 * (m - t) / t for m, t in zip(measured_dists, true_dists) if t != 0]

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))

    # Overlaid histograms of measured vs true
    ax1.hist(true_dists, bins=30, alpha=0.6, label='True', color='green')
    ax1.hist(measured_dists, bins=30, alpha=0.6, label='Measured', color='blue')
    ax1.set_xlabel('Distance')
    ax1.set_ylabel('Count')
    ax1.set_title('Measured vs True Range Distances')
    ax1.legend()

    # Error distribution
    ax2.hist(errors, bins=30, alpha=0.7, color='red')
    ax2.axvline(x=0, color='black', linestyle='--', linewidth=1)
    ax2.set_xlabel('Error (Measured - True)')
    ax2.set_ylabel('Count')
    ax2.set_title('Range Measurement Error Distribution')

    # Percent error distribution
    ax3.hist(pct_errors, bins=30, alpha=0.7, color='orange')
    ax3.axvline(x=0, color='black', linestyle='--', linewidth=1)
    ax3.set_xlabel('Percent Error (%)')
    ax3.set_ylabel('Count')
    ax3.set_title('Range Measurement Percent Error')

    plt.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_histogram.png"), dpi=300)
    plt.show()


def plot_range_diagnostics(fg_data, save_dir: str | None = None):
    """Plot range measurement error diagnostics broken down by association type.

    Categorizes range measurements into pose↔pose (robot-robot) and
    pose↔landmark (robot-landmark), then plots absolute error, percent error,
    and error vs measurement index for each category.

    Args:
        fg_data: FactorGraphData object with range measurements
        save_dir: Directory to save figure. If None, only shows interactively.
    """
    if not fg_data.range_measurements:
        print("No range measurements to diagnose.")
        return

    true_fg = make_all_ranges_perfect(fg_data)
    pose_keys = set(fg_data.pose_variables_dict.keys())

    categories = {
        "Robot ↔ Robot": {"measured": [], "true": []},
        "Robot ↔ Landmark": {"measured": [], "true": []},
    }

    for meas, true_meas in zip(fg_data.range_measurements, true_fg.range_measurements):
        name_a, name_b = meas.association
        a_is_pose = name_a in pose_keys
        b_is_pose = name_b in pose_keys

        if a_is_pose and b_is_pose:
            cat = "Robot ↔ Robot"
        elif a_is_pose or b_is_pose:
            cat = "Robot ↔ Landmark"
        else:
            continue

        categories[cat]["measured"].append(meas.dist)
        categories[cat]["true"].append(true_meas.dist)

    active_cats = {k: v for k, v in categories.items()
                   if len(v["measured"]) > 0}

    if not active_cats:
        print("No classifiable range measurements found.")
        return

    nrows = len(active_cats)
    fig, axes = plt.subplots(nrows, 3, figsize=(18, 5 * nrows))
    if nrows == 1:
        axes = axes[np.newaxis, :]

    for row, (cat_name, data) in enumerate(active_cats.items()):
        measured = np.array(data["measured"])
        true = np.array(data["true"])
        abs_err = measured - true
        pct_err = 100.0 * abs_err / true

        # Print summary
        print(f"\n{'=' * 50}")
        print(f"  {cat_name}  ({len(measured)} measurements)")
        print(f"{'=' * 50}")
        print(f"  Absolute error (m):  mean={np.mean(abs_err):.4f}  "
              f"std={np.std(abs_err):.4f}  max={np.max(np.abs(abs_err)):.4f}")
        print(f"  Percent error  (%):  mean={np.mean(pct_err):.2f}  "
              f"std={np.std(pct_err):.2f}  max={np.max(np.abs(pct_err)):.2f}")

        # Absolute error histogram
        ax = axes[row, 0]
        ax.hist(abs_err, bins=30, alpha=0.7, color='red')
        ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Absolute Error (m)")
        ax.set_ylabel("Count")
        ax.set_title(f"{cat_name} — Absolute Error")

        # Percent error histogram
        ax = axes[row, 1]
        ax.hist(pct_err, bins=30, alpha=0.7, color='orange')
        ax.axvline(x=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Percent Error (%)")
        ax.set_ylabel("Count")
        ax.set_title(f"{cat_name} — Percent Error")

        # Absolute error vs index
        ax = axes[row, 2]
        ax.scatter(range(len(abs_err)), abs_err, s=4, alpha=0.5, color='blue')
        ax.axhline(y=0, color='black', linestyle='--', linewidth=1)
        ax.set_xlabel("Measurement Index")
        ax.set_ylabel("Absolute Error (m)")
        ax.set_title(f"{cat_name} — Error vs Index")

    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_diagnostics.png"), dpi=300)
    plt.show()


def _split_range_errors(fg_data) -> dict[str, dict[str, np.ndarray]]:
    """Split range measurements into R↔R and R↔L populations.

    Returns a dict keyed by category name with np.ndarray values for
    'true_dist' (geometric range, m), 'abs_err' (signed metres,
    measured - true) and 'pct_err' (signed percent). Categories with
    no measurements are omitted.
    """
    true_fg = make_all_ranges_perfect(fg_data)
    pose_keys = set(fg_data.pose_variables_dict.keys())

    cats: dict[str, dict[str, list[float]]] = {
        "Robot ↔ Robot": {"true_dist": [], "abs_err": [], "pct_err": []},
        "Robot ↔ Landmark": {"true_dist": [], "abs_err": [], "pct_err": []},
    }
    for meas, true_meas in zip(fg_data.range_measurements,
                               true_fg.range_measurements):
        if true_meas.dist == 0:
            continue
        name_a, name_b = meas.association
        a_is_pose = name_a in pose_keys
        b_is_pose = name_b in pose_keys
        if a_is_pose and b_is_pose:
            cat = "Robot ↔ Robot"
        elif a_is_pose or b_is_pose:
            cat = "Robot ↔ Landmark"
        else:
            continue
        abs_err = meas.dist - true_meas.dist
        pct_err = 100.0 * abs_err / true_meas.dist
        cats[cat]["true_dist"].append(true_meas.dist)
        cats[cat]["abs_err"].append(abs_err)
        cats[cat]["pct_err"].append(pct_err)

    return {
        name: {k: np.asarray(v, dtype=float) for k, v in d.items()}
        for name, d in cats.items()
        if len(d["abs_err"]) > 0
    }


def _draw_distribution_axis(ax, values: np.ndarray, *,
                            color: str, title: str,
                            xlabel: str | None = None) -> None:
    """Histogram + mean/median/RMSE markers + zero-bias guide on `ax`."""
    ax.hist(values, bins=40, alpha=0.75, color=color, edgecolor="black",
            linewidth=0.4)

    mean_v = float(values.mean())
    median_v = float(np.median(values))
    rmse_v = float(np.sqrt(np.mean(values ** 2)))

    ax.axvline(0.0, color="0.4", linestyle="-", linewidth=0.8, alpha=0.6)
    ax.axvline(mean_v,   color="black", linestyle="-",  linewidth=1.2,
               label=f"mean = {mean_v:.3g}")
    ax.axvline(median_v, color="black", linestyle="--", linewidth=1.2,
               label=f"median = {median_v:.3g}")
    ax.axvline(rmse_v,   color="black", linestyle=":",  linewidth=1.2,
               label=f"RMSE = {rmse_v:.3g}")

    ax.set_title(f"{title}  (N = {values.size})")
    ax.set_ylabel("Count")
    if xlabel is not None:
        ax.set_xlabel(xlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)


def plot_range_output_distributions(fg_data,
                                    save_dir: str | None = None) -> None:
    """Thesis-clean range error distribution figures.

    Produces two PNGs in `save_dir`:
      - `range_abs_err_dist.png` — absolute error (m), R↔R top, R↔L bottom.
      - `range_rel_err_dist.png` — relative error (%), R↔R top, R↔L bottom.

    Each figure carries mean / median / RMSE marker lines per axis and a
    zero-bias guide. Distinct from the diagnostic outputs of
    `plot_range_histogram` / `plot_range_diagnostics` so both can coexist.
    """
    if not fg_data.range_measurements:
        print("No range measurements; skipping output distributions.")
        return

    cats = _split_range_errors(fg_data)
    if not cats:
        print("No classifiable range measurements; skipping output "
              "distributions.")
        return

    # Build the panel list once. Categories with no measurements are
    # dropped entirely so the figure shrinks rather than rendering a
    # placeholder axis.
    panels = []
    if "Robot ↔ Robot" in cats:
        panels.append(("Robot ↔ Robot", "tab:purple",
                       cats["Robot ↔ Robot"]))
    if "Robot ↔ Landmark" in cats:
        panels.append(("Robot ↔ Landmark", "tab:red",
                       cats["Robot ↔ Landmark"]))

    def _render(metric_key: str, xlabel: str, suptitle: str,
                save_name: str) -> None:
        n = len(panels)
        height = 4.0 if n == 1 else 6.5
        fig, axes = plt.subplots(n, 1, figsize=(9, height), sharex=True)
        if n == 1:
            axes = [axes]
        for ax, (title, color, data) in zip(axes, panels):
            _draw_distribution_axis(
                ax, data[metric_key], color=color,
                title=title,
                xlabel=xlabel if ax is axes[-1] else None)
        fig.suptitle(suptitle, fontsize=13)
        fig.tight_layout()
        if save_dir:
            fig.savefig(os.path.join(save_dir, save_name), dpi=300)

    _render("abs_err", "Range Error (m)", "Range Absolute Error",
            "range_abs_err_dist.png")
    _render("pct_err", "Range Error (%)", "Range Relative Error",
            "range_rel_err_dist.png")


def plot_range_bias_paper(fg_data,
                          save_dir: str | None = None,
                          robot_char: str | None = None) -> None:
    """Paper-quality signed-error histogram for robot-robot range bias.

    Single-panel figure showing bellhop-vs-straight-line range residuals
    for robot-robot links. The purpose is to make the systematic mean
    shift jump off the page. Empirical mean is drawn as a solid vertical
    line, zero-bias reference is drawn as a dashed line.

    Args:
        fg_data: FactorGraphData with range measurements.
        save_dir: Where to write the PNG.
        robot_char: If given, restrict to measurements where this robot
            is one of the two endpoints (e.g. "A"). Filename becomes
            `range_bias_paper_<char>.png`. If None, pool across all
            robot-robot links and write `range_bias_paper.png`.
    """
    if not fg_data.range_measurements:
        print("No range measurements; skipping bias paper plot.")
        return

    pose_keys = set(fg_data.pose_variables_dict.keys())
    true_fg = make_all_ranges_perfect(fg_data)

    errors_list = []
    for meas, true_meas in zip(fg_data.range_measurements,
                                true_fg.range_measurements):
        name_a, name_b = meas.association
        if not (name_a in pose_keys and name_b in pose_keys):
            continue
        if robot_char is not None:
            if name_a[0] != robot_char and name_b[0] != robot_char:
                continue
        errors_list.append(meas.dist - true_meas.dist)

    if not errors_list:
        label = f" for robot {robot_char!r}" if robot_char else ""
        print(f"No robot-robot range measurements{label}; "
              f"skipping bias paper plot.")
        return

    errors = np.asarray(errors_list, dtype=float)
    n = errors.size
    mean = float(errors.mean())
    std = float(errors.std(ddof=1)) if n > 1 else 0.0
    rmse = float(np.sqrt(np.mean(errors ** 2)))

    fig, ax = plt.subplots(figsize=(6.4, 3.8))

    ax.hist(
        errors, bins=60, color="#4c78a8",
        edgecolor="white", linewidth=0.4, alpha=0.9)

    ax.axvline(0.0, color="0.35", linestyle="--", linewidth=1.0,
               label="Zero bias")
    ax.axvline(mean, color="#d62728", linestyle="-", linewidth=1.4,
               label=f"Mean = {mean:+.2f} m")

    ax.set_xlabel(r"$r_{\mathrm{error}}$ (m)", fontsize=11)
    ax.set_ylabel("Count", fontsize=11)

    ax.grid(axis="y", which="major", linestyle="-", color="#333333",
            linewidth=0.9, alpha=0.55)
    ax.grid(axis="y", which="minor", linestyle="-", color="#888888",
            linewidth=0.4, alpha=0.25)
    ax.set_axisbelow(True)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    ax.tick_params(axis="both", which="both", length=3)

    ax.legend(frameon=True, fontsize=9, loc="upper left",
              handlelength=1.8, handletextpad=0.6, borderpad=0.4)

    fig.tight_layout()
    if save_dir:
        fname = (f"range_bias_paper_{robot_char}.png"
                 if robot_char is not None else "range_bias_paper.png")
        fig.savefig(os.path.join(save_dir, fname),
                    dpi=300, bbox_inches="tight")
    plt.close(fig)


def plot_range_error_vs_range(fg_data,
                              save_dir: str | None = None) -> None:
    """Thesis-clean scatter of |range error| vs. true range.

    Plots the magnitude of the bellhop range error so any trend with
    range is legible — signed errors cancel visually. Both axes show
    |error|: left = |measured - true| in metres, right = same in
    percent.

    Produces `range_err_vs_range.png` in `save_dir`. Two stacked
    subplots sharing the x-axis (R↔R top, R↔L bottom) auto-shrink
    when one population is empty. Each subplot has a twin y-axis:
    |absolute error| (m, blue, left) and |relative error| (%,
    orange, right). Y-axis labels, ticks, and spines are
    color-matched to their series so a print reader can attribute
    markers without consulting the legend.
    """
    if not fg_data.range_measurements:
        print("No range measurements; skipping range-vs-error scatter.")
        return

    cats = _split_range_errors(fg_data)
    if not cats:
        print("No classifiable range measurements; skipping "
              "range-vs-error scatter.")
        return

    panels = []
    if "Robot ↔ Robot" in cats:
        panels.append(("Robot ↔ Robot", cats["Robot ↔ Robot"]))
    if "Robot ↔ Landmark" in cats:
        panels.append(("Robot ↔ Landmark", cats["Robot ↔ Landmark"]))

    abs_color = "tab:blue"
    rel_color = "tab:orange"

    n = len(panels)
    height = 4.0 if n == 1 else 7.0
    fig, axes = plt.subplots(n, 1, figsize=(9, height), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, (title, data) in zip(axes, panels):
        true_d = data["true_dist"]
        abs_e = np.abs(data["abs_err"])
        pct_e = np.abs(data["pct_err"])

        ax.scatter(true_d, abs_e, s=14, alpha=0.5,
                   color=abs_color, label="|Absolute error| (m)",
                   edgecolor="none")
        ax.set_ylabel("|Absolute error| (m)", color=abs_color)
        ax.set_ylim(bottom=0.0)
        ax.tick_params(axis="y", labelcolor=abs_color)
        ax.spines["left"].set_color(abs_color)

        ax_r = ax.twinx()
        ax_r.scatter(true_d, pct_e, s=14, alpha=0.5,
                     color=rel_color, label="|Relative error| (%)",
                     edgecolor="none")
        ax_r.set_ylabel("|Relative error| (%)", color=rel_color)
        ax_r.set_ylim(bottom=0.0)
        ax_r.tick_params(axis="y", labelcolor=rel_color)
        ax_r.spines["right"].set_color(rel_color)
        ax_r.spines["left"].set_color(abs_color)

        ax.set_title(f"{title}  (N = {true_d.size})")
        ax.grid(True, axis="x", alpha=0.3)

        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = ax_r.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper left", fontsize=9)

    axes[-1].set_xlabel("True range (m)")
    fig.suptitle("Absolute Range Error vs. Range", fontsize=13)
    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_err_vs_range.png"),
                    dpi=300)


def _split_range_measurements(fg_data) -> dict[str, np.ndarray]:
    """Split measured range distances into R↔R and R↔L populations.

    Companion to `_split_range_errors` for the raw measurement view.
    Returns a dict keyed by category with a 1D numpy array of measured
    distances; categories with no measurements are omitted so the
    downstream plotter can drop empty panels.
    """
    pose_keys = set(fg_data.pose_variables_dict.keys())
    cats: dict[str, list[float]] = {
        "Robot ↔ Robot": [],
        "Robot ↔ Landmark": [],
    }
    for meas in fg_data.range_measurements:
        name_a, name_b = meas.association
        a_is_pose = name_a in pose_keys
        b_is_pose = name_b in pose_keys
        if a_is_pose and b_is_pose:
            cats["Robot ↔ Robot"].append(float(meas.dist))
        elif a_is_pose or b_is_pose:
            cats["Robot ↔ Landmark"].append(float(meas.dist))
    return {name: np.asarray(v, dtype=float)
            for name, v in cats.items() if v}


def _draw_range_axis(ax, values: np.ndarray, *,
                     color: str, title: str,
                     xlabel: str | None = None) -> None:
    """Histogram of measured ranges + min/max/mean/median markers.

    Sibling to `_draw_distribution_axis`; stats chosen to *frame* the
    ranging envelope (operational extent) rather than centering on a
    zero-bias residual.
    """
    ax.hist(values, bins=40, alpha=0.75, color=color, edgecolor="black",
            linewidth=0.4)

    min_v = float(values.min())
    max_v = float(values.max())
    mean_v = float(values.mean())
    median_v = float(np.median(values))

    ax.axvline(min_v,    color="black", linestyle="-",  linewidth=1.4,
               label=f"min = {min_v:.0f} m")
    ax.axvline(max_v,    color="black", linestyle="-",  linewidth=1.4,
               label=f"max = {max_v:.0f} m")
    ax.axvline(mean_v,   color="0.3",   linestyle="--", linewidth=1.2,
               label=f"mean = {mean_v:.0f} m")
    ax.axvline(median_v, color="0.3",   linestyle=":",  linewidth=1.2,
               label=f"median = {median_v:.0f} m")

    ax.set_title(f"{title}  (N = {values.size})")
    ax.set_ylabel("Count")
    if xlabel is not None:
        ax.set_xlabel(xlabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)


def plot_range_measurement_distribution(fg_data,
                                        save_dir: str | None = None) -> None:
    """Thesis-clean histogram of bellhop range measurements.

    Produces `range_measurement_dist.png` in `save_dir`. Two stacked
    subplots sharing the x-axis: R↔R on top (purple), R↔L on bottom
    (red). Min / max / mean / median vertical markers per axis frame
    the operational ranging envelope of the simulation. Empty
    populations are dropped — the figure shrinks rather than rendering
    a placeholder.
    """
    if not fg_data.range_measurements:
        print("No range measurements; skipping range-distribution plot.")
        return

    cats = _split_range_measurements(fg_data)
    if not cats:
        print("No classifiable range measurements; skipping range-"
              "distribution plot.")
        return

    panels = []
    if "Robot ↔ Robot" in cats:
        panels.append(("Robot ↔ Robot", "tab:purple",
                       cats["Robot ↔ Robot"]))
    if "Robot ↔ Landmark" in cats:
        panels.append(("Robot ↔ Landmark", "tab:red",
                       cats["Robot ↔ Landmark"]))

    n = len(panels)
    height = 4.0 if n == 1 else 6.5
    fig, axes = plt.subplots(n, 1, figsize=(9, height), sharex=True)
    if n == 1:
        axes = [axes]
    for ax, (title, color, data) in zip(axes, panels):
        _draw_range_axis(
            ax, data, color=color, title=title,
            xlabel="Measured Range (m)" if ax is axes[-1] else None)
    fig.suptitle("Bellhop Measured Range Distribution", fontsize=13)
    fig.tight_layout()
    if save_dir:
        fig.savefig(os.path.join(save_dir, "range_measurement_dist.png"),
                    dpi=300)


def print_worst_ranges(fg_data, n: int = 20):
    """Print a table of range measurements with the largest percent error.

    Columns: timestamp, source, receiver, measured, true, abs_err, pct_err
    Sorted by descending absolute percent error.
    """
    if not fg_data.range_measurements:
        print("No range measurements.")
        return

    true_fg = make_all_ranges_perfect(fg_data)

    rows = []
    for meas, true_meas in zip(fg_data.range_measurements, true_fg.range_measurements):
        if true_meas.dist == 0:
            continue
        abs_err = meas.dist - true_meas.dist
        pct_err = 100.0 * abs_err / true_meas.dist
        rows.append((
            meas.timestamp if meas.timestamp is not None else float('nan'),
            meas.association[0],
            meas.association[1],
            meas.dist,
            true_meas.dist,
            abs_err,
            pct_err,
        ))

    rows.sort(key=lambda r: abs(r[6]), reverse=True)

    print(f"\nTop {min(n, len(rows))} range measurements by percent error:")
    print(f"  {'Time':>10s}  {'Source':>8s}  {'Receiver':>8s}  "
          f"{'Measured':>10s}  {'True':>10s}  {'AbsErr':>10s}  {'PctErr':>8s}")
    print(f"  {'─' * 10}  {'─' * 8}  {'─' * 8}  "
          f"{'─' * 10}  {'─' * 10}  {'─' * 10}  {'─' * 8}")
    for row in rows[:n]:
        ts, src, rcv, measured, true, abs_e, pct_e = row
        print(f"  {ts:10.3f}  {src:>8s}  {rcv:>8s}  "
              f"{measured:10.4f}  {true:10.4f}  {abs_e:+10.4f}  {pct_e:+7.2f}%")


def plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True,
                         save_dir: str | None = None):
    """Plot 3D factor graph data.

    Args:
        fg_data: FactorGraphData object (must be 3D)
        show_trajectories: Whether to plot pose trajectories
        show_landmarks: Whether to plot landmark variables
        save_dir: Directory to save figure. If None, only shows interactively.
    """

    if fg_data.dimension != 3:
        raise ValueError(f"Data is {fg_data.dimension}D, not 3D")

    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Invert z-axis so positive z points downward (depth)
    ax.invert_zaxis()

    # Plot landmarks
    if show_landmarks and fg_data.landmark_variables:
        print(f"Plotting {len(fg_data.landmark_variables)} landmarks...")
        for landmark in fg_data.landmark_variables:
            draw_landmark_variable_3d(ax, landmark)

    # Plot range measurements
    if fg_data.range_measurements:
        print(f"Plotting {len(fg_data.range_measurements)} range measurements...")
        all_variables = fg_data.pose_and_landmark_variables_dict
        pose_variables = fg_data.pose_variables_dict
        skipped_ranges = 0

        for range_meas in fg_data.range_measurements:
            key_1, key_2 = range_meas.association
            var_1 = all_variables.get(key_1)
            var_2 = all_variables.get(key_2)

            # Skip malformed associations that do not resolve to known variables.
            if var_1 is None or var_2 is None:
                skipped_ranges += 1
                continue

            # draw_range_measurement_3d needs one endpoint to be a pose.
            if key_1 in pose_variables:
                from_pose = var_1
                to_variable = var_2
            elif key_2 in pose_variables:
                from_pose = var_2
                to_variable = var_1
            else:
                skipped_ranges += 1
                continue

            if not isinstance(from_pose, PoseVariable3D):
                skipped_ranges += 1
                continue

            draw_range_measurement_3d(
                ax,
                range_meas,
                from_pose,
                to_variable,
                add_line=True,
            )

        if skipped_ranges:
            print(f"Skipped {skipped_ranges} range measurements (no pose endpoint or unresolved key).")

    # Plot pose variables and trajectories
    print(f"Plotting {fg_data.num_robots} robot trajectories...")
    colors = ['blue', 'red', 'green', 'orange', 'purple']

    for robot_idx in range(fg_data.num_robots):
        color = colors[robot_idx % len(colors)]
        poses = fg_data.pose_variables[robot_idx]

        if show_trajectories and len(poses) > 0:
            # Extract x, y, z coordinates from poses
            x_traj = [pose.true_x for pose in poses]
            y_traj = [pose.true_y for pose in poses]
            z_traj = [pose.true_z for pose in poses]

            # Draw trajectory line
            draw_traj_3d(ax, x_traj, y_traj, z_traj, color=color)

            # Draw some poses along the trajectory (every Nth pose to avoid clutter)
            step = max(1, len(poses) // 10)
            for pose in poses[::step]:
                draw_pose_3d(ax, pose, color=color, scale=0.5)

    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'3D Factor Graph - {fg_data.num_robots} Robot(s), {len(fg_data.landmark_variables)} Landmarks')

    if save_dir:
        fig.savefig(os.path.join(save_dir, "factor_graph_3d.png"), dpi=300)
    plt.show()


if __name__ == "__main__":
    # Read the factor graph
    print(f"Reading .pyfg file: {FILE_PATH}")
    fg_data = read_from_pyfg_text(FILE_PATH)

    # Print some info
    print(f"\nFactor Graph Info:")
    print(f"  Dimension: {fg_data.dimension}D")
    print(f"  Robots: {fg_data.num_robots}")
    print(f"  Landmarks: {len(fg_data.landmark_variables)}")
    print(f"  Poses: {sum(len(poses) for poses in fg_data.pose_variables)}")
    print(f"  Range measurements: {len(fg_data.range_measurements)}")

    print_worst_ranges(fg_data)

    # --- Debug diagnostics (cluttered, not for thesis) ---
    # plot_range_histogram(fg_data, save_dir=WORK_DIR)
    # plot_range_diagnostics(fg_data, save_dir=WORK_DIR)

    # --- Thesis-clean outputs ---
    plot_range_output_distributions(fg_data, save_dir=WORK_DIR)
    plot_range_measurement_distribution(fg_data, save_dir=WORK_DIR)
    plot_range_error_vs_range(fg_data, save_dir=WORK_DIR)

    plot_factor_graph_3d(fg_data, show_trajectories=True, show_landmarks=True,
                         save_dir=WORK_DIR)
    plot_range_errors_vtk(fg_data, save_dir=WORK_DIR, show_landmark_hull=True)
