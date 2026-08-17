#!/usr/bin/env python3
"""Parse sim_log.txt to build LaTeX tables of range-ping success/failure.

Two tables are emitted to stdout (also written as .tex files if TEX_OUT_DIR
is set):

  1. Overall status summary: one row per RangeStatus enum with count and %.
  2. Per-pair failure counts: one row per (pinger, target) with attempts,
     failures, and success rate. Sorted by failure count descending.

Edit the constants at the top and run:  `uv run python report_range_failures.py`
"""

import os
import re
from collections import Counter

# SIM_LOG = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
#            "src/results/arctic/beaufort-fleet-week/sim_log.txt")
SIM_LOG = ("/home/tko/repos/manta-ray/mantaray/cmake-build-release/"
           "src/results/arctic/fram-strait-fleet-week/sim_log.txt")
TEX_OUT_DIR: str | None = os.path.dirname(SIM_LOG)  # None disables .tex writes

# Regexes for the ping-status lines the C++ side emits via SPDLOG.
# Format: "[<level>] [t=<t> <pinger>[R]-><target>[R]] <message>"
_TAG_RE = re.compile(
    r"\[t=(?P<t>[\d.]+)\s+(?P<pinger>\d+)\[[RL]\]->(?P<target>\d+)\[[RL]\]\]"
)
_STATUS_PATTERNS = [
    ("kOk",                r"Ping OK"),
    ("kNoArrival",         r"Ping dropped: no arrival"),
    ("kOutOfBounds",       r"Ping dropped: (pinger|target|both).*out of bounds"),
    ("kSspSampleFailed",   r"Ping dropped: SSP sample failed"),
    ("kSkippedPingerDead", r"Ping dropped: pinger .* dead"),
    ("kSkippedTargetDead", r"Ping dropped: target .* dead"),
]
_STATUS_HUMAN = {
    "kOk":                "Success",
    "kNoArrival":         "No arrival (shadow zone / no direct path)",
    "kOutOfBounds":       "Endpoint out of bounds",
    "kSspSampleFailed":   "SSP sample failed",
    "kSkippedPingerDead": "Skipped (pinger dead)",
    "kSkippedTargetDead": "Skipped (target dead)",
}


def parse_sim_log(path: str):
    """Return (status_counts, pair_stats) tuple.

    status_counts: {status_str: count}
    pair_stats:    {(pinger, target): {"attempts": n, "ok": n, "fail": n}}
    """
    status_counts: Counter[str] = Counter()
    pair_stats: dict[tuple[int, int], dict[str, int]] = {}

    with open(path) as f:
        for line in f:
            status = None
            for name, pat in _STATUS_PATTERNS:
                if re.search(pat, line):
                    status = name
                    break
            if status is None:
                continue

            status_counts[status] += 1

            tag = _TAG_RE.search(line)
            if tag is None:
                continue
            key = (int(tag["pinger"]), int(tag["target"]))
            slot = pair_stats.setdefault(
                key, {"attempts": 0, "ok": 0, "fail": 0})
            slot["attempts"] += 1
            if status == "kOk":
                slot["ok"] += 1
            else:
                slot["fail"] += 1

    return status_counts, pair_stats


def format_summary_latex(status_counts: Counter[str]) -> str:
    """Two-column LaTeX table: status | count | percent."""
    total = sum(status_counts.values())
    rows = []
    # Preserve the order in _STATUS_HUMAN so failure rows sit under Success.
    for name, human in _STATUS_HUMAN.items():
        n = status_counts.get(name, 0)
        if n == 0:
            continue
        pct = 100.0 * n / total if total else 0.0
        rows.append((human, n, pct))

    lines = [
        r"\begin{table}[t]",
        r"  \centering",
        r"  \caption{Acoustic ranging outcomes over the mission. "
        r"``No arrival'' groups shadow-zone drops and any ping where the "
        r"iterative beam search exhausted \texttt{max\_beams} without "
        r"finding a direct-path arrival.}",
        r"  \label{tab:range_status_summary}",
        r"  \begin{tabular}{lrr}",
        r"    \toprule",
        r"    Outcome & Count & Fraction (\%) \\",
        r"    \midrule",
    ]
    for human, n, pct in rows:
        lines.append(f"    {human} & {n:,} & {pct:.2f} \\\\")
    lines += [
        r"    \midrule",
        f"    Total attempts & {total:,} & 100.00 \\\\",
        r"    \bottomrule",
        r"  \end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def format_pair_latex(pair_stats: dict, top_n: int | None = 20) -> str:
    """LaTeX longtable of per-pair attempts/failures. Sorted by failures desc."""
    rows = sorted(
        pair_stats.items(),
        key=lambda kv: (-kv[1]["fail"], kv[0][0], kv[0][1]))
    if top_n is not None:
        rows = rows[:top_n]

    top_label = (f" (top {top_n} pairs by failure count)"
                 if top_n is not None else "")
    lines = [
        r"\begin{table}[t]",
        r"  \centering",
        rf"  \caption{{Per-pair acoustic ranging outcomes{top_label}. "
        r"Failure = any drop status. Success rate = OK / attempts.}",
        r"  \label{tab:range_status_by_pair}",
        r"  \begin{tabular}{ccrrr}",
        r"    \toprule",
        r"    Pinger & Target & Attempts & Failures & Success (\%) \\",
        r"    \midrule",
    ]
    for (p, t), s in rows:
        rate = 100.0 * s["ok"] / s["attempts"] if s["attempts"] else 0.0
        lines.append(
            f"    R{p} & R{t} & {s['attempts']:,} & {s['fail']:,} "
            f"& {rate:.1f} \\\\")
    lines += [
        r"    \bottomrule",
        r"  \end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def main() -> None:
    if not os.path.exists(SIM_LOG):
        raise FileNotFoundError(f"sim_log.txt not found: {SIM_LOG}")

    status_counts, pair_stats = parse_sim_log(SIM_LOG)
    total = sum(status_counts.values())
    print(f"# Parsed {total:,} status lines from {SIM_LOG}")
    for name in _STATUS_HUMAN:
        n = status_counts.get(name, 0)
        if n:
            print(f"  {_STATUS_HUMAN[name]:45s}: {n:>6,d} "
                  f"({100 * n / total:.2f}%)")
    print(f"  {'Unique (pinger, target) pairs seen':45s}: "
          f"{len(pair_stats):>6,d}")

    summary_tex = format_summary_latex(status_counts)
    pair_tex = format_pair_latex(pair_stats)

    print("\n" + "=" * 70)
    print("SUMMARY TABLE")
    print("=" * 70)
    print(summary_tex)
    print("\n" + "=" * 70)
    print("PER-PAIR TABLE (top 20 by failure count)")
    print("=" * 70)
    print(pair_tex)

    if TEX_OUT_DIR:
        os.makedirs(TEX_OUT_DIR, exist_ok=True)
        p_sum = os.path.join(TEX_OUT_DIR, "range_status_summary.tex")
        p_pair = os.path.join(TEX_OUT_DIR, "range_status_by_pair.tex")
        with open(p_sum, "w") as f:
            f.write(summary_tex + "\n")
        with open(p_pair, "w") as f:
            f.write(pair_tex + "\n")
        print(f"\nWrote {p_sum}")
        print(f"Wrote {p_pair}")


if __name__ == "__main__":
    main()
