#!/usr/bin/env python3
"""
Diagnostics for PyFactorGraph data — GPS prior validation.

Compares GPS pose priors against ground-truth pose positions to check
whether the observed error statistics match the configured GPS noise
and whether any priors are misassociated.
"""

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import numpy as np
from py_factor_graph.factor_graph import FactorGraphData
from py_factor_graph.priors import PosePrior3D
from py_factor_graph.utils.name_utils import get_time_idx_from_frame_name


def get_gps_priors(fg: FactorGraphData) -> list[PosePrior3D]:
    """Return only GPS priors, excluding the first-pose prior per robot.

    The first-pose prior (time index 0) is an exact ground-truth anchor,
    not a GPS measurement.
    """
    first_pose_names = set()
    for pose_chain in fg.pose_variables:
        if pose_chain:
            first_pose_names.add(pose_chain[0].name)

    return [
        p for p in fg.pose_priors
        if isinstance(p, PosePrior3D) and p.name not in first_pose_names
    ]


def validate_gps_priors(fg: FactorGraphData) -> dict:
    """Compare GPS prior positions against ground-truth poses.

    Returns a dict with per-prior errors, aggregate statistics,
    and expected vs observed noise for sanity checking.
    """
    gps_priors = get_gps_priors(fg)
    if not gps_priors:
        return {"n": 0}

    pose_dict = fg.pose_variables_dict

    errors_xyz = []
    errors_dist = []
    prior_names = []
    pose_indices = []
    expected_sigmas = []

    for prior in gps_priors:
        gt_pose = pose_dict.get(prior.name)
        if gt_pose is None:
            continue

        err = np.array(prior.position) - np.array(gt_pose.true_position)
        errors_xyz.append(err)
        errors_dist.append(np.linalg.norm(err))
        prior_names.append(prior.name)
        pose_indices.append(get_time_idx_from_frame_name(prior.name))
        expected_sigmas.append(1.0 / np.sqrt(prior.translation_precision))

    errors_xyz = np.array(errors_xyz)
    errors_dist = np.array(errors_dist)
    expected_sigma = float(np.median(expected_sigmas))

    return {
        "n": len(prior_names),
        "errors_xyz": errors_xyz,
        "errors_dist": errors_dist,
        "expected_sigma": expected_sigma,
        "observed_sigma_xyz": tuple(errors_xyz.std(axis=0)),
        "observed_mean_xyz": tuple(errors_xyz.mean(axis=0)),
        "prior_names": prior_names,
        "pose_indices": np.array(pose_indices),
    }


def print_gps_report(stats: dict):
    """Pretty-print GPS prior validation statistics."""
    if stats["n"] == 0:
        print("No GPS priors found.")
        return

    n = stats["n"]
    sigma = stats["expected_sigma"]
    mean = stats["observed_mean_xyz"]
    std = stats["observed_sigma_xyz"]
    dist = stats["errors_dist"]

    print(f"\nGPS Prior Validation ({n} priors)")
    print(f"  Expected sigma (from precision): {sigma:.4f} m")
    print(f"  {'Axis':<6s} {'Mean (m)':>12s} {'Obs Std (m)':>12s}")
    for i, axis in enumerate(("x", "y", "z")):
        print(f"  {axis:<6s} {mean[i]:12.6f} {std[i]:12.6f}")
    print(f"  {'dist':<6s} {dist.mean():12.6f} {dist.std():12.6f}")

    outlier_thresh = 3.0 * sigma
    outlier_mask = dist > outlier_thresh
    n_outliers = outlier_mask.sum()
    print(f"\n  Outliers (>{3.0 * sigma:.3f} m): {n_outliers}/{n}")
    if n_outliers > 0:
        names = [stats["prior_names"][i] for i in np.where(outlier_mask)[0]]
        for name in names[:20]:
            idx = stats["prior_names"].index(name)
            print(f"    {name}: {dist[idx]:.4f} m")
        if n_outliers > 20:
            print(f"    ... and {n_outliers - 20} more")


def plot_gps_errors(stats: dict):
    """Plot GPS prior error diagnostics.

    4 subplots:
      1. Per-axis error histograms with expected sigma lines
      2. 3D distance error histogram
      3. Error magnitude vs pose time index
      4. Per-axis error vs pose time index
    """
    if stats["n"] == 0:
        print("No GPS priors to plot.")
        return

    errors_xyz = stats["errors_xyz"]
    errors_dist = stats["errors_dist"]
    sigma = stats["expected_sigma"]
    indices = stats["pose_indices"]

    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    ax = axes[0, 0]
    colors = ["tab:red", "tab:green", "tab:blue"]
    for i, (axis, c) in enumerate(zip(("x", "y", "z"), colors)):
        ax.hist(errors_xyz[:, i], bins=40, alpha=0.5, color=c, label=axis)
    ax.axvline(-sigma, color="black", linestyle="--", linewidth=1, label=f"±{sigma:.3f} m")
    ax.axvline(sigma, color="black", linestyle="--", linewidth=1)
    ax.set_xlabel("Error (m)")
    ax.set_ylabel("Count")
    ax.set_title("Per-Axis Error Distribution")
    ax.legend()

    ax = axes[0, 1]
    ax.hist(errors_dist, bins=40, color="tab:orange", alpha=0.7)
    ax.axvline(sigma, color="black", linestyle="--", linewidth=1, label=f"σ = {sigma:.3f} m")
    ax.axvline(3 * sigma, color="red", linestyle="--", linewidth=1, label=f"3σ = {3*sigma:.3f} m")
    ax.set_xlabel("3D Distance Error (m)")
    ax.set_ylabel("Count")
    ax.set_title("Distance Error Distribution")
    ax.legend()

    ax = axes[1, 0]
    ax.scatter(indices, errors_dist, s=4, alpha=0.6, color="tab:purple")
    ax.axhline(sigma, color="black", linestyle="--", linewidth=1)
    ax.axhline(3 * sigma, color="red", linestyle="--", linewidth=1)
    ax.set_xlabel("Pose Time Index")
    ax.set_ylabel("3D Distance Error (m)")
    ax.set_title("Error Magnitude vs Pose Index")

    ax = axes[1, 1]
    for i, (axis, c) in enumerate(zip(("x", "y", "z"), colors)):
        ax.plot(indices, errors_xyz[:, i], linewidth=0.6, alpha=0.7, color=c, label=axis)
    ax.axhline(0, color="black", linewidth=0.5)
    ax.set_xlabel("Pose Time Index")
    ax.set_ylabel("Error (m)")
    ax.set_title("Per-Axis Error vs Pose Index")
    ax.legend()

    fig.suptitle("GPS Prior vs Ground Truth Diagnostics", fontsize=14)
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    from py_factor_graph.io.pyfg_text import read_from_pyfg_text

    FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src/output.pfg"

    print(f"Reading {FILE_PATH} ...")
    fg = read_from_pyfg_text(FILE_PATH)

    stats = validate_gps_priors(fg)
    print_gps_report(stats)
    plot_gps_errors(stats)
