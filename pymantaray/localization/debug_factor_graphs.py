#!/usr/bin/env python3
"""Per-factor residual breakdown and 3D visualization for solved factor graphs.

Exposes `debug_factor_graph(solver, save_dir, prefix)` — prints per-type
importance tables + top offenders, saves matplotlib breakdown/histogram
figures, and opens an interactive PyVista scene showing factors in the
world with on-screen residual labels for the worst ones.
"""

import os
from copy import deepcopy
from dataclasses import dataclass, field

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np

import gtsam


# ───────────────────── factor classification ─────────────────────

FACTOR_TYPE_GPS_PRIOR = "GPS prior"
FACTOR_TYPE_LANDMARK_PRIOR = "Landmark prior"
FACTOR_TYPE_BETWEEN = "Odometry/Between"
FACTOR_TYPE_RANGE_PP = "Range pose↔pose"
FACTOR_TYPE_RANGE_PL = "Range pose↔landmark"
FACTOR_TYPE_RANGE_LL = "Range landmark↔landmark"
FACTOR_TYPE_DEPTH = "Depth prior (custom)"


def classify_factor(factor) -> str:
    """Return a short human label for a GTSAM factor."""
    if isinstance(factor, gtsam.PriorFactorPose3):
        return FACTOR_TYPE_GPS_PRIOR
    if isinstance(factor, gtsam.PriorFactorPoint3):
        return FACTOR_TYPE_LANDMARK_PRIOR
    if isinstance(factor, gtsam.BetweenFactorPose3):
        return FACTOR_TYPE_BETWEEN
    if isinstance(factor, gtsam.RangeFactorPose3):
        return FACTOR_TYPE_RANGE_PP
    if isinstance(factor, gtsam.RangeFactor3D):
        return FACTOR_TYPE_RANGE_PL
    if isinstance(factor, gtsam.RangeFactor3):
        return FACTOR_TYPE_RANGE_LL
    if isinstance(factor, gtsam.CustomFactor):
        return FACTOR_TYPE_DEPTH
    return type(factor).__name__


# ───────────────────── stats computation ─────────────────────

@dataclass
class FactorRecord:
    """Single-factor snapshot used for ranking / plotting / labeling."""
    index: int
    type_label: str
    endpoints: tuple[str, ...]
    whitened: float
    raw: float
    dof: int = 1
    leverage: float = float("nan")
    cook: float = float("nan")


@dataclass
class FactorTypeStats:
    type_label: str
    records: list[FactorRecord] = field(default_factory=list)

    @property
    def count(self) -> int:
        return len(self.records)

    @property
    def all_whitened(self) -> np.ndarray:
        return np.array([r.whitened for r in self.records], dtype=float)

    @property
    def all_raw(self) -> np.ndarray:
        return np.array([r.raw for r in self.records], dtype=float)

    @property
    def sum_whitened(self) -> float:
        return float(self.all_whitened.sum()) if self.records else 0.0

    @property
    def mean_whitened(self) -> float:
        return float(self.all_whitened.mean()) if self.records else 0.0

    @property
    def median_whitened(self) -> float:
        return float(np.median(self.all_whitened)) if self.records else 0.0

    @property
    def max_whitened(self) -> float:
        return float(self.all_whitened.max()) if self.records else 0.0

    @property
    def mean_raw(self) -> float:
        return float(self.all_raw.mean()) if self.records else 0.0

    @property
    def median_raw(self) -> float:
        return float(np.median(self.all_raw)) if self.records else 0.0

    @property
    def max_raw(self) -> float:
        return float(self.all_raw.max()) if self.records else 0.0


def _build_reverse_key_map(solver) -> dict[int, str]:
    return {key: name for name, key in solver.key_map.items()}


def _raw_norm(factor, values) -> float:
    """L2 norm of unwhitened residual. Returns NaN if factor has no such hook."""
    try:
        r = factor.unwhitenedError(values)
    except Exception:
        return float("nan")
    arr = np.asarray(r, dtype=float).ravel()
    if arr.size == 0:
        return float("nan")
    return float(np.linalg.norm(arr))


def compute_factor_stats(graph, values, reverse_key_map: dict[int, str]
                         ) -> dict[str, FactorTypeStats]:
    """Walk the graph once, group by classify_factor(), collect per-factor records."""
    stats: dict[str, FactorTypeStats] = {}
    for i in range(graph.size()):
        f = graph.at(i)
        if f is None:
            continue
        label = classify_factor(f)
        keys = list(f.keys())
        endpoints = tuple(reverse_key_map.get(k, f"key:{k}") for k in keys)

        whitened = float(f.error(values))
        raw = _raw_norm(f, values)
        try:
            dof = int(f.dim())
        except Exception:
            dof = 1
        if dof <= 0:
            dof = 1

        rec = FactorRecord(index=i, type_label=label, endpoints=endpoints,
                           whitened=whitened, raw=raw, dof=dof)
        stats.setdefault(label, FactorTypeStats(type_label=label)).records.append(rec)
    return stats


# ───────────────────── stdlib box-drawn table ─────────────────────

def _render_table(headers: list[str],
                  rows: list[list[str]],
                  aligns: list[str],
                  total_row: list[str] | None = None,
                  title: str | None = None) -> str:
    """Render a table with ┌─┬─┐ borders and dynamic column widths.

    aligns: list of "l" / "r" per column.
    total_row: optional final row preceded by a separator rule.
    title: optional centered title rendered above the top rule.
    """
    assert len(headers) == len(aligns)
    all_rows = list(rows) + ([total_row] if total_row else [])
    widths = [max(len(h), max((len(r[i]) for r in all_rows), default=0))
              for i, h in enumerate(headers)]

    def fmt_cell(text: str, w: int, align: str) -> str:
        return f" {text:>{w}} " if align == "r" else f" {text:<{w}} "

    def fmt_row(cells: list[str]) -> str:
        return "│" + "│".join(fmt_cell(c, widths[i], aligns[i])
                              for i, c in enumerate(cells)) + "│"

    def rule(left: str, mid: str, right: str) -> str:
        return left + mid.join("─" * (w + 2) for w in widths) + right

    lines: list[str] = []
    total_width = sum(widths) + 3 * len(widths) + 1
    if title:
        lines.append(title.center(total_width))
    lines.append(rule("┌", "┬", "┐"))
    lines.append(fmt_row(headers))
    lines.append(rule("├", "┼", "┤"))
    for r in rows:
        lines.append(fmt_row(r))
    if total_row:
        lines.append(rule("├", "┼", "┤"))
        lines.append(fmt_row(total_row))
    lines.append(rule("└", "┴", "┘"))
    return "\n".join(lines)


# ───────────────────── console tables ─────────────────────

def _fmt_float(x: float, width: int = 10, decimals: int = 4) -> str:
    if not np.isfinite(x):
        return "nan".rjust(width)
    return f"{x:,.{decimals}f}"


def print_factor_importance_table(stats: dict[str, FactorTypeStats],
                                  total_error: float,
                                  prefix: str = "") -> None:
    """Sum of whitened error per factor type, sorted by importance."""
    if not stats:
        print("No factors in graph.")
        return

    ordered = sorted(stats.values(), key=lambda s: s.sum_whitened, reverse=True)

    headers = ["Type", "Count", "Σ whitened", "% total",
               "mean", "median", "max"]
    aligns = ["l", "r", "r", "r", "r", "r", "r"]
    rows = []
    total_count = 0
    total_whitened = 0.0
    for s in ordered:
        pct = (s.sum_whitened / total_error * 100.0) if total_error > 0 else 0.0
        rows.append([
            s.type_label,
            f"{s.count:,}",
            f"{s.sum_whitened:,.2f}",
            f"{pct:.1f}%",
            _fmt_float(s.mean_whitened),
            _fmt_float(s.median_whitened),
            _fmt_float(s.max_whitened),
        ])
        total_count += s.count
        total_whitened += s.sum_whitened

    total_row = [
        "TOTAL",
        f"{total_count:,}",
        f"{total_whitened:,.2f}",
        "100.0%" if total_error > 0 else "—",
        "", "", "",
    ]

    title = f"Factor importance — {prefix}" if prefix else "Factor importance"
    print()
    print(_render_table(headers, rows, aligns, total_row=total_row, title=title))
    print()


def print_top_offenders(stats: dict[str, FactorTypeStats],
                        k: int = 20,
                        prefix: str = "") -> list[FactorRecord]:
    """Global top-K factors by whitened error, across all types."""
    all_records: list[FactorRecord] = []
    for s in stats.values():
        all_records.extend(s.records)
    all_records.sort(key=lambda r: r.whitened, reverse=True)
    top = all_records[:k]
    if not top:
        return top

    headers = ["Rank", "Type", "Endpoints", "raw", "whitened"]
    aligns = ["r", "l", "l", "r", "r"]
    rows = []
    for i, r in enumerate(top, start=1):
        endpoints = " ↔ ".join(r.endpoints) if len(r.endpoints) > 1 \
            else r.endpoints[0]
        rows.append([
            str(i),
            r.type_label,
            endpoints,
            _fmt_float(r.raw),
            _fmt_float(r.whitened),
        ])

    title = (f"Top {len(top)} worst factors — {prefix}" if prefix
             else f"Top {len(top)} worst factors")
    print(_render_table(headers, rows, aligns, title=title))
    print()
    return top


# ───────────────────── matplotlib plots ─────────────────────

def plot_factor_breakdown(stats: dict[str, FactorTypeStats],
                          save_dir: str | None = None,
                          prefix: str = "") -> None:
    """Bar chart (Σ whitened per type) + pie of share of total cost."""
    if not stats:
        return
    ordered = sorted(stats.values(), key=lambda s: s.sum_whitened, reverse=True)
    labels = [s.type_label for s in ordered]
    sums = np.array([s.sum_whitened for s in ordered])
    counts = [s.count for s in ordered]

    fig, (ax_bar, ax_pie) = plt.subplots(1, 2, figsize=(14, 6))

    bars = ax_bar.barh(labels, sums, color="tab:blue")
    ax_bar.invert_yaxis()
    ax_bar.set_xlabel("Σ whitened error")
    ax_bar.set_title(f"Per-type whitened sum ({prefix})")
    for bar, c in zip(bars, counts):
        ax_bar.text(bar.get_width(), bar.get_y() + bar.get_height() / 2,
                    f"  n={c:,}", va="center", fontsize=9)

    ax_pie.pie(sums, labels=labels, autopct="%1.1f%%", startangle=90,
               colors=plt.cm.tab10.colors)
    ax_pie.set_title("Share of total cost")

    fig.tight_layout()
    if save_dir:
        path = os.path.join(save_dir, f"{prefix}_factor_breakdown.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"Saved {path}")


def plot_residual_distributions(stats: dict[str, FactorTypeStats],
                                save_dir: str | None = None,
                                prefix: str = "") -> None:
    """One subplot per factor type: raw vs whitened histograms on log-y."""
    if not stats:
        return
    ordered = sorted(stats.values(), key=lambda s: s.sum_whitened, reverse=True)
    n = len(ordered)
    cols = min(3, n)
    rows = (n + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(5 * cols, 4 * rows),
                             squeeze=False)

    for ax, s in zip(axes.flat, ordered):
        raw = s.all_raw[np.isfinite(s.all_raw)]
        whitened = s.all_whitened[np.isfinite(s.all_whitened)]
        bins = 40
        if raw.size:
            ax.hist(raw, bins=bins, alpha=0.5, label="raw", color="tab:blue")
        if whitened.size:
            ax.hist(whitened, bins=bins, alpha=0.5, label="whitened",
                    color="tab:orange")
        ax.set_yscale("log")
        ax.set_title(f"{s.type_label} (n={s.count:,})")
        ax.set_xlabel("residual")
        ax.set_ylabel("count (log)")
        ax.legend(fontsize=8)

    # Hide any trailing empty axes.
    for ax in axes.flat[n:]:
        ax.set_visible(False)

    fig.suptitle(f"Raw vs whitened residuals — {prefix}", fontsize=14)
    fig.tight_layout()
    if save_dir:
        path = os.path.join(save_dir, f"{prefix}_residual_distributions.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"Saved {path}")


# ───────────────────── marginal leverage (opt-in) ─────────────────────

def compute_factor_leverages(solver, values) -> tuple[dict[int, float],
                                                      "gtsam.Marginals | None"]:
    """Per-factor marginal leverage tr(A · Σ_local · Aᵀ).

    Σ_local is the posterior marginal covariance of the variables this
    factor touches, computed conditional on every other factor in the
    graph. Cost: one global Cholesky (shared across factors) plus one
    back-solve per factor — opt-in only.

    Returns (leverages_by_factor_index, marginals_object). The Marginals
    object is returned so callers can reuse it for pose-marginal plots
    without re-factorizing.
    """
    graph = solver.graph
    try:
        marginals = gtsam.Marginals(graph, values)
    except RuntimeError as e:
        print(f"[leverage] gtsam.Marginals failed (non-PD system?): {e}")
        return {}, None

    n = graph.size()
    print(f"[leverage] computing {n:,} factor leverages...")
    out: dict[int, float] = {}
    for i in range(n):
        f = graph.at(i)
        if f is None:
            continue
        try:
            gf = f.linearize(values)
            A, _ = gf.jacobian()
            keys = list(f.keys())
            if not keys:
                continue
            kv = gtsam.KeyVector(keys)
            Sigma = marginals.jointMarginalCovariance(kv).fullMatrix()
            out[i] = float(np.trace(A @ Sigma @ A.T))
        except Exception:
            out[i] = float("nan")
    return out, marginals


def _attach_leverages_to_stats(stats: dict[str, "FactorTypeStats"],
                               leverages: dict[int, float]) -> None:
    """Fill FactorRecord.leverage in place from an index→leverage map."""
    for s in stats.values():
        for rec in s.records:
            rec.leverage = leverages.get(rec.index, float("nan"))


def print_leverage_importance_table(stats: dict[str, "FactorTypeStats"],
                                    prefix: str = "") -> None:
    """Per-type leverage summary: count, Σ, mean, median, max. Mirrors the
    whitened-error importance table so the two can be compared side by side.
    """
    type_arrays: list[tuple[str, np.ndarray]] = []
    for s in stats.values():
        arr = np.array([r.leverage for r in s.records
                        if np.isfinite(r.leverage)], dtype=float)
        if arr.size:
            type_arrays.append((s.type_label, arr))
    if not type_arrays:
        return

    total_sum = sum(float(a.sum()) for _, a in type_arrays)
    type_arrays.sort(key=lambda t: t[1].max(), reverse=True)

    headers = ["Type", "Count", "Σ leverage", "% total",
               "mean", "median", "max"]
    aligns = ["l", "r", "r", "r", "r", "r", "r"]
    rows = []
    total_count = 0
    for label, arr in type_arrays:
        s_sum = float(arr.sum())
        pct = (s_sum / total_sum * 100.0) if total_sum > 0 else 0.0
        rows.append([
            label,
            f"{arr.size:,}",
            f"{s_sum:,.4g}",
            f"{pct:.1f}%",
            f"{float(arr.mean()):,.4g}",
            f"{float(np.median(arr)):,.4g}",
            f"{float(arr.max()):,.4g}",
        ])
        total_count += arr.size

    total_row = [
        "TOTAL",
        f"{total_count:,}",
        f"{total_sum:,.4g}",
        "100.0%" if total_sum > 0 else "—",
        "", "", "",
    ]
    title = (f"Leverage by factor type — {prefix}" if prefix
             else "Leverage by factor type")
    print()
    print(_render_table(headers, rows, aligns, total_row=total_row, title=title))
    print()


def print_top_leverages(stats: dict[str, "FactorTypeStats"],
                        k: int = 20,
                        prefix: str = "") -> list["FactorRecord"]:
    """Box-drawn table of the top-K factors by marginal leverage."""
    all_records: list[FactorRecord] = []
    for s in stats.values():
        all_records.extend(r for r in s.records if np.isfinite(r.leverage))
    if not all_records:
        return []
    all_records.sort(key=lambda r: r.leverage, reverse=True)
    top = all_records[:k]

    headers = ["Rank", "Type", "Endpoints", "leverage", "whitened", "raw"]
    aligns = ["r", "l", "l", "r", "r", "r"]
    rows = []
    for i, r in enumerate(top, start=1):
        endpoints = " ↔ ".join(r.endpoints) if len(r.endpoints) > 1 \
            else r.endpoints[0]
        rows.append([
            str(i),
            r.type_label,
            endpoints,
            f"{r.leverage:,.4g}",
            _fmt_float(r.whitened),
            _fmt_float(r.raw),
        ])

    title = (f"Top {len(top)} factors by leverage — {prefix}" if prefix
             else f"Top {len(top)} factors by leverage")
    print(_render_table(headers, rows, aligns, title=title))
    print()
    return top


# ───────────────────── influence (Cook's distance) ─────────────────────

def _attach_cook_to_stats(stats: dict[str, "FactorTypeStats"]) -> None:
    """Fill FactorRecord.cook in place: D = (h/dof) * (whitened/(1-h)).

    Requires `leverage` already populated. Clamps (1-h) away from zero so
    a saturated leverage doesn't produce inf.
    """
    eps = 1e-9
    for s in stats.values():
        for rec in s.records:
            h = rec.leverage
            if not np.isfinite(h) or not np.isfinite(rec.whitened):
                continue
            denom = max(1.0 - h, eps)
            rec.cook = (h / max(rec.dof, 1)) * (rec.whitened / denom)


def print_influence_importance_table(stats: dict[str, "FactorTypeStats"],
                                     prefix: str = "") -> None:
    """Per-type Cook's-distance summary: count, Σ, mean, median, max."""
    type_arrays: list[tuple[str, np.ndarray]] = []
    for s in stats.values():
        arr = np.array([r.cook for r in s.records
                        if np.isfinite(r.cook)], dtype=float)
        if arr.size:
            type_arrays.append((s.type_label, arr))
    if not type_arrays:
        return

    total_sum = sum(float(a.sum()) for _, a in type_arrays)
    type_arrays.sort(key=lambda t: float(t[1].sum()), reverse=True)

    headers = ["Type", "Count", "Σ Cook", "% total",
               "mean", "median", "max"]
    aligns = ["l", "r", "r", "r", "r", "r", "r"]
    rows = []
    total_count = 0
    for label, arr in type_arrays:
        s_sum = float(arr.sum())
        pct = (s_sum / total_sum * 100.0) if total_sum > 0 else 0.0
        rows.append([
            label,
            f"{arr.size:,}",
            f"{s_sum:,.4g}",
            f"{pct:.1f}%",
            f"{float(arr.mean()):,.4g}",
            f"{float(np.median(arr)):,.4g}",
            f"{float(arr.max()):,.4g}",
        ])
        total_count += arr.size

    total_row = [
        "TOTAL",
        f"{total_count:,}",
        f"{total_sum:,.4g}",
        "100.0%" if total_sum > 0 else "—",
        "", "", "",
    ]
    title = (f"Factor influence (Cook's D) — {prefix}" if prefix
             else "Factor influence (Cook's D)")
    print()
    print(_render_table(headers, rows, aligns, total_row=total_row, title=title))
    print()


def print_top_influence(stats: dict[str, "FactorTypeStats"],
                        k: int = 20,
                        prefix: str = "") -> list["FactorRecord"]:
    """Box-drawn table of the top-K factors by Cook's distance."""
    all_records: list[FactorRecord] = []
    for s in stats.values():
        all_records.extend(r for r in s.records if np.isfinite(r.cook))
    if not all_records:
        return []
    all_records.sort(key=lambda r: r.cook, reverse=True)
    top = all_records[:k]

    headers = ["Rank", "Type", "Endpoints", "Cook", "leverage", "whitened", "raw"]
    aligns = ["r", "l", "l", "r", "r", "r", "r"]
    rows = []
    for i, r in enumerate(top, start=1):
        endpoints = " ↔ ".join(r.endpoints) if len(r.endpoints) > 1 \
            else r.endpoints[0]
        rows.append([
            str(i),
            r.type_label,
            endpoints,
            f"{r.cook:,.4g}",
            f"{r.leverage:,.4g}",
            _fmt_float(r.whitened),
            _fmt_float(r.raw),
        ])

    title = (f"Top {len(top)} most influential factors — {prefix}" if prefix
             else f"Top {len(top)} most influential factors")
    print(_render_table(headers, rows, aligns, title=title))
    print()
    return top


def plot_pose_marginals(solver,
                        marginals: "gtsam.Marginals",
                        save_dir: str | None = None,
                        prefix: str = "") -> None:
    """Position-uncertainty traces along the pose chain, plus GPS-priored
    pose markers. Reuses an existing Marginals object — no extra Cholesky.
    """
    pose_chains = solver.fg.pose_variables
    if not pose_chains:
        return

    # Find GPS-priored pose names by scanning the graph for PriorFactorPose3.
    gps_pose_names: set[str] = set()
    reverse_map = _build_reverse_key_map(solver)
    for i in range(solver.graph.size()):
        f = solver.graph.at(i)
        if isinstance(f, gtsam.PriorFactorPose3):
            ks = list(f.keys())
            if ks:
                gps_pose_names.add(reverse_map.get(ks[0], ""))

    fig, (ax_sig, ax_tr) = plt.subplots(2, 1, figsize=(12, 7), sharex=True)

    robot_colors = ["tab:blue", "tab:red", "tab:green", "tab:purple",
                    "tab:orange"]
    any_plotted = False
    for r_idx, chain in enumerate(pose_chains):
        if not chain:
            continue
        xs, sx, sy, sz, tr_vals = [], [], [], [], []
        gps_marks: list[int] = []
        color = robot_colors[r_idx % len(robot_colors)]
        for p_idx, pose_var in enumerate(chain):
            key = solver.key_map.get(pose_var.name)
            if key is None:
                continue
            try:
                cov = marginals.marginalCovariance(key)
            except Exception:
                continue
            # Pose3 marginal is 6×6 [rot(3), trans(3)] — grab translation block.
            pos_cov = np.asarray(cov)[3:6, 3:6]
            diag = np.clip(np.diag(pos_cov), 0.0, None)
            xs.append(p_idx)
            sx.append(np.sqrt(diag[0]))
            sy.append(np.sqrt(diag[1]))
            sz.append(np.sqrt(diag[2]))
            tr_vals.append(float(np.trace(pos_cov)))
            if pose_var.name in gps_pose_names:
                gps_marks.append(p_idx)

        if not xs:
            continue
        any_plotted = True
        ax_sig.plot(xs, sx, "-", color=color, alpha=0.9, label=f"r{r_idx} σx")
        ax_sig.plot(xs, sy, "--", color=color, alpha=0.6, label=f"r{r_idx} σy")
        ax_sig.plot(xs, sz, ":", color=color, alpha=0.6, label=f"r{r_idx} σz")
        ax_tr.plot(xs, tr_vals, "-", color=color, label=f"robot {r_idx}")
        for gx in gps_marks:
            ax_sig.axvline(gx, color=color, alpha=0.15, linewidth=1)
            ax_tr.axvline(gx, color=color, alpha=0.15, linewidth=1)

    if not any_plotted:
        plt.close(fig)
        return

    ax_sig.set_ylabel("position stddev (m)")
    ax_sig.set_title("Per-axis position uncertainty (GPS-priored poses marked)")
    ax_sig.legend(fontsize=8, ncol=3)
    ax_sig.grid(True, alpha=0.3)
    ax_tr.set_ylabel("tr(Σ_pos) (m²)")
    ax_tr.set_xlabel("pose index")
    ax_tr.set_title("Total position uncertainty")
    ax_tr.legend(fontsize=8)
    ax_tr.grid(True, alpha=0.3)

    fig.suptitle(f"Pose marginals — {prefix}", fontsize=14)
    fig.tight_layout()
    if save_dir:
        path = os.path.join(save_dir, f"{prefix}_pose_marginals.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"Saved {path}")


# ───────────────────── prior-health panel ─────────────────────

def _pose_index_from_name(name: str) -> int | None:
    """Parse trailing integer from a variable name like 'A342' → 342."""
    digits = ""
    for ch in reversed(name):
        if ch.isdigit():
            digits = ch + digits
        else:
            break
    return int(digits) if digits else None


def plot_prior_health(stats: dict[str, "FactorTypeStats"],
                      save_dir: str | None = None,
                      prefix: str = "") -> None:
    """2D plot of prior residuals vs variable index, one subplot per type.

    Priors are cluttered in the 3D view (one per pose for depth, one per
    landmark, etc.). A 1-D "residual vs index" scan shows drift patterns
    and outliers at a glance.
    """
    prior_types = [FACTOR_TYPE_GPS_PRIOR, FACTOR_TYPE_LANDMARK_PRIOR,
                   FACTOR_TYPE_DEPTH]
    present = [t for t in prior_types if t in stats and stats[t].count]
    if not present:
        return

    fig, axes = plt.subplots(len(present), 1, figsize=(12, 3.2 * len(present)),
                             squeeze=False)
    for ax, type_label in zip(axes[:, 0], present):
        s = stats[type_label]
        xs, raw_y, whi_y, labels = [], [], [], []
        for rec in s.records:
            ep = rec.endpoints[0] if rec.endpoints else ""
            idx = _pose_index_from_name(ep)
            xs.append(idx if idx is not None else len(xs))
            raw_y.append(rec.raw)
            whi_y.append(rec.whitened)
            labels.append(ep)
        order = np.argsort(xs)
        xs_arr = np.array(xs)[order]
        raw_arr = np.array(raw_y)[order]
        whi_arr = np.array(whi_y)[order]

        ax.plot(xs_arr, whi_arr, ".-", color="tab:orange",
                label="whitened", markersize=4, linewidth=1)
        ax.set_ylabel("whitened", color="tab:orange")
        ax.tick_params(axis="y", labelcolor="tab:orange")
        ax.set_xlabel(f"{type_label} — variable index")

        ax2 = ax.twinx()
        ax2.plot(xs_arr, raw_arr, ".", color="tab:blue", alpha=0.5,
                 label="raw", markersize=3)
        ax2.set_ylabel("raw residual", color="tab:blue")
        ax2.tick_params(axis="y", labelcolor="tab:blue")

        ax.set_title(f"{type_label} (n={s.count:,}, "
                     f"Σ whitened={s.sum_whitened:,.2f})")
        ax.grid(True, alpha=0.3)

    fig.suptitle(f"Prior health — {prefix}", fontsize=14)
    fig.tight_layout()
    if save_dir:
        path = os.path.join(save_dir, f"{prefix}_prior_health.png")
        fig.savefig(path, dpi=150, bbox_inches="tight")
        print(f"Saved {path}")


# All PyVista/VTK plotting for the localization package lives in `vtk_plots.py`.
# The top-level entry point below imports from it lazily (vtk_plots itself
# imports FactorRecord / type constants from this module, so a top-level
# import here would be circular).


# ───────────────────── top-level entry point ─────────────────────

def debug_factor_graph(solver,
                       save_dir: str | None = None,
                       prefix: str = "",
                       top_k: int = 20,
                       label_k: int = 50,
                       show_3d: bool = True,
                       show_leverage: bool = False) -> dict[str, FactorTypeStats]:
    """Run all factor-graph diagnostics on a solved FactorGraphSolver.

    `show_leverage=True` runs the opt-in marginal-leverage pass (one
    global Cholesky + N back-solves) and opens a second PyVista window
    + a pose-marginal matplotlib panel. Expensive — leave off in the
    hot debug loop.
    """
    if solver.result is None:
        print("[debug_factor_graphs] solver.result is None; call .solve() first.")
        return {}

    reverse_map = _build_reverse_key_map(solver)
    stats = compute_factor_stats(solver.graph, solver.result, reverse_map)
    total_error = float(solver.graph.error(solver.result))

    print(f"\n=== Factor-graph diagnostics — {prefix} ===")
    print(f"Total graph error (0.5 Σ ‖r_whitened‖²): {total_error:,.4f}")

    print_factor_importance_table(stats, total_error, prefix=prefix)
    print_top_offenders(stats, k=top_k, prefix=prefix)

    plot_factor_breakdown(stats, save_dir=save_dir, prefix=prefix)
    plot_residual_distributions(stats, save_dir=save_dir, prefix=prefix)
    plot_prior_health(stats, save_dir=save_dir, prefix=prefix)

    marginals_obj = None
    if show_leverage:
        leverages, marginals_obj = compute_factor_leverages(
            solver, solver.result)
        if leverages:
            _attach_leverages_to_stats(stats, leverages)
            print_leverage_importance_table(stats, prefix=prefix)
            print_top_leverages(stats, k=top_k, prefix=prefix)
            _attach_cook_to_stats(stats)
            print_influence_importance_table(stats, prefix=prefix)
            print_top_influence(stats, k=top_k, prefix=prefix)
        if marginals_obj is not None:
            plot_pose_marginals(solver, marginals_obj,
                                save_dir=save_dir, prefix=prefix)

    # Render matplotlib figures non-blocking so the user can study them,
    # then launch the 3D view(s) on top.
    if show_3d:
        plt.show(block=False)
        plt.pause(0.1)

    from vtk_plots import plot_factors_in_world, plot_leverage_in_world
    plot_factors_in_world(solver, stats, save_dir=save_dir, prefix=prefix,
                          label_k=label_k, show=show_3d)
    if show_leverage and marginals_obj is not None:
        plot_leverage_in_world(solver, stats, save_dir=save_dir,
                               prefix=prefix, label_k=label_k, show=show_3d)
    return stats


# ───────────────────── standalone entry ─────────────────────

if __name__ == "__main__":
    from py_factor_graph.io.pyfg_text import read_from_pyfg_text

    from pyfg_to_gtsam import (FactorGraphSolver, SolverConfig,
                               odom_cadence_from_fg)

    FILE_PATH = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/results/arctic/beaufort-floats-long/output.pfg"
    WORK_DIR = os.path.dirname(FILE_PATH)

    default_pos_prior = 0.1
    angular_noise = 1e-6
    xy_frac = 0.05
    z_frac = 0.01
    odom_noise = np.array([angular_noise, angular_noise, angular_noise,
                           xy_frac, xy_frac, z_frac])
    odom_gtsam_noise = deepcopy(odom_noise)
    odom_gtsam_noise[:3] = 1e-2

    gps_prior_sigmas = np.array(
        [2, 2, 2, default_pos_prior, default_pos_prior, default_pos_prior],
        dtype=np.float64)

    print(f"Reading {FILE_PATH} ...")
    fg_data = read_from_pyfg_text(FILE_PATH)
    odom_cadence_dt = odom_cadence_from_fg(fg_data)

    config = SolverConfig(
        odom_noise_sigmas=odom_noise,
        range_noise_stddev=1.0,
        include_ranges=True,
        between_noise_sigmas=odom_gtsam_noise,
        landmark_prior_sigma=default_pos_prior,
        gps_prior_sigmas=gps_prior_sigmas,
        depth_prior_sigma=0.01 / 3.0,
        depth_prior_mode="custom",
        odom_cadence_dt=odom_cadence_dt,
        odom_drift_rate_trans=0.01,
        odom_drift_rate_rot=1e-6,
    )

    solver = FactorGraphSolver(fg_data, config)
    solver.solve()
    debug_factor_graph(solver, save_dir=WORK_DIR, prefix="measured")
    plt.show()
