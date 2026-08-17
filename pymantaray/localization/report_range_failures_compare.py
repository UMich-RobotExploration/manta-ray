#!/usr/bin/env python3
"""Cross-environment range-drop comparison.

Parses two sim_log.txt files (e.g. Beaufort vs Fram Strait) and emits a
LaTeX table breaking down failures by link category:

  * Surface-Surface (S<->S)  — both endpoints are surface floats
  * Surface-Diver   (S<->D)  — one surface float, one diver
  * Diver-Diver     (D<->D)  — both endpoints are divers

Makes the narrative point that the two environments fail for different
reasons: Beaufort concentrates drops in surface-to-surface (halocline
duct), while Fram Strait spreads drops across more categories due to
broader SSP variability.

Edit ENV_CONFIGS and SURFACE_INDICES, then:
    uv run python report_range_failures_compare.py
"""

import os
import re
from collections import Counter

# One tuple per environment: (short_label, path/to/sim_log.txt).
ENV_CONFIGS = [
    ("Beaufort",
     "/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
     "src/results/arctic/beaufort-fleet-week/sim_log.txt"),
    ("Fram Strait",
     "/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
     "src/results/arctic/fram-strait-fleet-week/sim_log.txt"),
]

# Robot indices that are surface floats (from beaufort_fleet_week_sim.json
# and fram_strait_fleet_week_sim.json — first two entries).
SURFACE_INDICES = {0, 1}

# Where to write the .tex output (also printed to stdout).
TEX_OUT_PATH: str | None = ("/home/tko/repos/manta-ray/mantaray/"
                            "cmake-build-release/src/results/arctic/"
                            "range_drop_category_compare.tex")

_TAG_RE = re.compile(
    r"\[t=(?P<t>[\d.]+)\s+(?P<pinger>\d+)\[[RL]\]->(?P<target>\d+)\[[RL]\]\]"
)
_STATUS_OK   = re.compile(r"Ping OK")
_STATUS_DROP = re.compile(r"Ping dropped")


def classify(a: int, b: int) -> str:
    """Return 'S-S', 'S-D', or 'D-D' for a link between robot indices a, b."""
    a_surf = a in SURFACE_INDICES
    b_surf = b in SURFACE_INDICES
    if a_surf and b_surf:
        return "S-S"
    if a_surf or b_surf:
        return "S-D"
    return "D-D"


def parse(log_path: str) -> dict[str, dict[str, int]]:
    """Return {category: {'attempts': n, 'fail': n}}."""
    stats: dict[str, dict[str, int]] = {
        c: {"attempts": 0, "fail": 0} for c in ("S-S", "S-D", "D-D")
    }
    with open(log_path) as f:
        for line in f:
            is_ok = bool(_STATUS_OK.search(line))
            is_drop = bool(_STATUS_DROP.search(line))
            if not (is_ok or is_drop):
                continue
            tag = _TAG_RE.search(line)
            if not tag:
                continue
            cat = classify(int(tag["pinger"]), int(tag["target"]))
            stats[cat]["attempts"] += 1
            if is_drop:
                stats[cat]["fail"] += 1
    return stats


_CATEGORY_HUMAN = {
    "S-S": "Surface $\\leftrightarrow$ Surface",
    "S-D": "Surface $\\leftrightarrow$ Diver",
    "D-D": "Diver $\\leftrightarrow$ Diver",
}


def format_compare_latex(env_names: list[str],
                         env_stats: list[dict]) -> str:
    """LaTeX table: one row per link category, one column per environment.

    Only the failure percentage is shown -- raw counts and attempt totals
    live in the caption text if needed. Prioritises visual clarity.
    """
    # Manual ruled table (no booktabs). Vertical bars + \hline only.
    col_spec = "|l" + "|r" * len(env_names) + "|"
    header = " & ".join(["Link category"] + env_names) + r" \\"

    lines = [
        r"\begin{table}[t]",
        r"  \centering",
        rf"  \begin{{tabular}}{{{col_spec}}}",
        r"    \hline",
        f"    {header}",
        r"    \hline",
    ]

    for cat in _CATEGORY_HUMAN:
        att = env_stats[0][cat]["attempts"]
        cells = [_CATEGORY_HUMAN[cat]]
        for stats in env_stats:
            rate = 100.0 * stats[cat]["fail"] / att if att else 0.0
            cells.append(f"{rate:.1f}")
        lines.append("    " + " & ".join(cells) + r" \\")

    # Total row.
    total_att = sum(env_stats[0][c]["attempts"] for c in _CATEGORY_HUMAN)
    total_cells = ["All links"]
    for stats in env_stats:
        total_fail = sum(stats[c]["fail"] for c in _CATEGORY_HUMAN)
        rate = 100.0 * total_fail / total_att if total_att else 0.0
        total_cells.append(f"{rate:.1f}")

    lines += [
        r"    \hline",
        "    " + " & ".join(total_cells) + r" \\",
        r"    \hline",
        r"  \end{tabular}",
        r"  \caption{Ranging failure rate (\%) by link category. Both "
        r"environments share the same fleet layout and ping schedule "
        r"(5{,}544 total attempts each); only the environmental sound-speed "
        r"structure differs.}",
        r"  \label{tab:range_drop_category_compare}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def main() -> None:
    env_names = [name for name, _ in ENV_CONFIGS]
    env_stats = []
    for name, path in ENV_CONFIGS:
        if not os.path.exists(path):
            raise FileNotFoundError(f"sim_log missing for {name}: {path}")
        env_stats.append(parse(path))

    # Human-readable stdout summary.
    header = "Category".ljust(10) + " Attempts"
    for n in env_names:
        header += f"   {n:>18s}"
    print(header)
    for cat in _CATEGORY_HUMAN:
        att = env_stats[0][cat]["attempts"]
        line = f"{cat:<10s}  {att:>8,}"
        for stats in env_stats:
            f = stats[cat]["fail"]
            rate = 100.0 * f / att if att else 0.0
            line += f"   {f:>6,d} ({rate:5.2f}%)"
        print(line)
    total_att = sum(env_stats[0][c]["attempts"] for c in _CATEGORY_HUMAN)
    line = "TOTAL     " + f"  {total_att:>8,}"
    for stats in env_stats:
        f = sum(stats[c]["fail"] for c in _CATEGORY_HUMAN)
        rate = 100.0 * f / total_att if total_att else 0.0
        line += f"   {f:>6,d} ({rate:5.2f}%)"
    print(line)

    print("\n" + "=" * 70)
    tex = format_compare_latex(env_names, env_stats)
    print(tex)

    if TEX_OUT_PATH:
        os.makedirs(os.path.dirname(TEX_OUT_PATH), exist_ok=True)
        with open(TEX_OUT_PATH, "w") as f:
            f.write(tex + "\n")
        print(f"\nWrote {TEX_OUT_PATH}")


if __name__ == "__main__":
    main()
