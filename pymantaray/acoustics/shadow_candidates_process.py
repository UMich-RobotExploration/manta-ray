"""Rank `noarrival_*.env` candidates by source/receiver depth delta.

Scans RESULTS_DIR for every Bellhop env file the simulator dumped whenever a ping
produced no arrivals, pulls out source and receiver geometry, and sorts the set
by |z_src - z_rcv| descending — the assumption being that the largest vertical
offsets are the best a-priori candidates for refraction-driven acoustic shadow
zones. For the top TOP_N it also renders an env-only vertical-slice preview
(bathymetry along src→rcv bearing + src/rcv markers) so the geometry can be
eyeballed before the user commits to a full 3D ray-trace run.

Edit RESULTS_DIR and TOP_N below; no CLI arguments.
"""

import csv
import glob
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import RegularGridInterpolator

from readers_1_8 import read_bty_3d

# ---- Edit these ----
RESULTS_DIR = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long"
TOP_N = 5
NOARRIVAL_GLOB = "noarrival_*.env"


def _val(line: str) -> str:
    return line.split("!")[0].strip()


def parse_env(env_path: str):
    """Parse source/receiver positions and beam fan from a 3D .env file.

    Inlined from debug_raytrace.py to avoid pulling in its open3d import chain.
    Fixed 0-indexed line layout:
      8: SX km, 10: SY km, 12: SD m, 14: RD m, 16: R km, 18: bearing deg,
      21: alpha_min alpha_max, 23: beta_min beta_max.
    """
    with open(env_path) as f:
        lines = f.readlines()

    sx = float(_val(lines[8]).split()[0]) * 1000.0
    sy = float(_val(lines[10]).split()[0]) * 1000.0
    sd = float(_val(lines[12]).split()[0])
    rd = float(_val(lines[14]).split()[0])
    rcv_r = float(_val(lines[16]).split()[0]) * 1000.0
    bearing = float(_val(lines[18]).split()[0])

    src_pos = np.array([sx, sy, sd])
    bearing_rad = np.radians(bearing)
    rcv_x = sx + rcv_r * np.cos(bearing_rad)
    rcv_y = sy + rcv_r * np.sin(bearing_rad)
    rcv_pos = np.array([rcv_x, rcv_y, rd])

    alpha_parts = _val(lines[21]).replace("/", "").split()
    alpha_range = (float(alpha_parts[0]), float(alpha_parts[1]))

    beta_parts = _val(lines[23]).replace("/", "").split()
    beta_range = (float(beta_parts[0]), float(beta_parts[1]))

    return src_pos, rcv_pos, alpha_range, beta_range


def scan_candidates(results_dir: str) -> list[dict]:
    """Parse every noarrival_*.env in results_dir and return rows sorted
    by |SD - RD| descending. Each row is a self-contained dict carrying
    everything print_ranked_table / write_ranking_csv / plot_env_preview
    need, so the main pipeline stays a flat list comprehension."""
    env_paths = sorted(glob.glob(os.path.join(results_dir, NOARRIVAL_GLOB)))
    rows: list[dict] = []
    for env_path in env_paths:
        src_pos, rcv_pos, alpha_range, beta_range = parse_env(env_path)
        horiz_range = float(np.linalg.norm(rcv_pos[:2] - src_pos[:2]))
        depth_delta = float(abs(src_pos[2] - rcv_pos[2]))
        dy = rcv_pos[1] - src_pos[1]
        dx = rcv_pos[0] - src_pos[0]
        bearing = float(np.degrees(np.arctan2(dy, dx)))
        rows.append({
            "name": Path(env_path).stem,
            "env_path": env_path,
            "src_pos": src_pos,
            "rcv_pos": rcv_pos,
            "depth_delta": depth_delta,
            "horiz_range": horiz_range,
            "bearing": bearing,
            "alpha_range": alpha_range,
            "beta_range": beta_range,
        })
    rows.sort(key=lambda r: r["depth_delta"], reverse=True)
    return rows


def print_ranked_table(rows: list[dict]) -> None:
    headers = ["#", "name", "SD (m)", "RD (m)", "|dz| (m)", "range (km)", "bearing (°)"]
    data = []
    for i, r in enumerate(rows, start=1):
        data.append([
            str(i),
            r["name"],
            f"{r['src_pos'][2]:.2f}",
            f"{r['rcv_pos'][2]:.2f}",
            f"{r['depth_delta']:.2f}",
            f"{r['horiz_range'] / 1000.0:.3f}",
            f"{r['bearing']:.2f}",
        ])

    widths = [max(len(h), *(len(row[i]) for row in data)) for i, h in enumerate(headers)]

    def fmt_row(cells, left, mid, right):
        parts = [" " + c.ljust(w) + " " for c, w in zip(cells, widths)]
        return left + mid.join(parts) + right

    top = "┌" + "┬".join("─" * (w + 2) for w in widths) + "┐"
    sep = "├" + "┼".join("─" * (w + 2) for w in widths) + "┤"
    bot = "└" + "┴".join("─" * (w + 2) for w in widths) + "┘"

    print(top)
    print(fmt_row(headers, "│", "│", "│"))
    print(sep)
    for row in data:
        print(fmt_row(row, "│", "│", "│"))
    print(bot)


def write_ranking_csv(rows: list[dict], out_path: Path) -> None:
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow([
            "rank", "name", "env_path",
            "sd_m", "rd_m", "depth_delta_m",
            "horiz_range_m", "bearing_deg",
            "src_x_m", "src_y_m", "rcv_x_m", "rcv_y_m",
            "alpha_min_deg", "alpha_max_deg",
            "beta_min_deg", "beta_max_deg",
        ])
        for i, r in enumerate(rows, start=1):
            w.writerow([
                i, r["name"], r["env_path"],
                r["src_pos"][2], r["rcv_pos"][2], r["depth_delta"],
                r["horiz_range"], r["bearing"],
                r["src_pos"][0], r["src_pos"][1],
                r["rcv_pos"][0], r["rcv_pos"][1],
                r["alpha_range"][0], r["alpha_range"][1],
                r["beta_range"][0], r["beta_range"][1],
            ])


def plot_env_preview(row: dict, save_path: Path) -> None:
    """Vertical-slice preview of a single env: bathymetry along the src→rcv
    bearing, sea surface, source/receiver markers, and the straight-line
    direct path. Mirrors the bathy+markers half of debug_raytrace.plot_rays_2d
    but without rays (which don't exist until Bellhop is run in ray mode)."""
    src_pos = row["src_pos"]
    rcv_pos = row["rcv_pos"]
    d = row["horiz_range"]
    if d < 1e-6:
        print(f"  [preview] {row['name']}: src and rcv share xy — skipping")
        return

    u = (rcv_pos[:2] - src_pos[:2]) / d

    bty_path = Path(row["env_path"]).with_suffix(".bty")
    bty = read_bty_3d(str(bty_path)) if bty_path.exists() else None

    fig, ax = plt.subplots(figsize=(12, 5))

    if bty is not None:
        x_vals = np.asarray(bty["ranges"])
        y_vals = np.asarray(bty["crossranges"])
        Z = np.asarray(bty["depths"])
        interp = RegularGridInterpolator(
            (x_vals, y_vals), Z, method="linear", bounds_error=False,
            fill_value=None)
        r_line = np.linspace(-0.1 * d, 1.3 * d, 400)
        sample_xy = src_pos[:2][None, :] + np.outer(r_line, u)
        try:
            z_bot = interp(sample_xy)
            ax.plot(r_line, z_bot, color="saddlebrown", linewidth=1.5,
                    label="bathymetry (along bearing)")
            ax.fill_between(r_line, z_bot, np.nanmax(z_bot) + 100,
                            color="saddlebrown", alpha=0.15)
        except Exception as e:
            print(f"  [preview] {row['name']}: bathymetry interp failed: {e}")

    ax.axhline(0.0, color="steelblue", linewidth=1.2, alpha=0.6,
               label="sea surface")
    ax.plot([0.0, d], [src_pos[2], rcv_pos[2]],
            linestyle="--", color="black", linewidth=1.6, alpha=0.8,
            label="direct src↔rcv", zorder=4)
    ax.plot(0.0, src_pos[2], marker="o", color="red", markersize=16,
            markeredgecolor="black", markeredgewidth=1.2,
            label="source", zorder=6)
    ax.plot(d, rcv_pos[2], marker="o", color="limegreen", markersize=16,
            markeredgecolor="black", markeredgewidth=1.2,
            label="receiver", zorder=6)

    ax.invert_yaxis()
    ax.set_xlabel("horizontal range from source (m)", fontsize=13)
    ax.set_ylabel("depth (m)", fontsize=13)
    ax.set_title(
        f"{row['name']}   |dz|={row['depth_delta']:.1f} m   "
        f"range={d / 1000.0:.2f} km   bearing={row['bearing']:.1f}°",
        fontsize=13,
    )
    ax.grid(True, alpha=0.3)
    ax.legend(loc="lower right", fontsize=11)
    fig.tight_layout()
    fig.savefig(save_path, dpi=150, bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    rows = scan_candidates(RESULTS_DIR)
    if not rows:
        print(f"No {NOARRIVAL_GLOB} files found in {RESULTS_DIR}")
        raise SystemExit(1)

    print(f"Found {len(rows)} no-arrival envs in {RESULTS_DIR}\n")
    print_ranked_table(rows)

    csv_path = Path(RESULTS_DIR) / "noarrival_ranking.csv"
    write_ranking_csv(rows, csv_path)
    print(f"\nWrote {csv_path}")

    for row in rows[:TOP_N]:
        png = Path(RESULTS_DIR) / f"{row['name']}_envpreview.png"
        plot_env_preview(row, png)
        print(f"Saved {png}")

    top_root = Path(RESULTS_DIR) / rows[0]["name"]
    print(f"\nTop candidate to run next: {rows[0]['name']}")
    print(f'  file_root = "{top_root}"')
