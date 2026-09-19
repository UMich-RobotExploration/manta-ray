"""Guard against evo/seaborn silently mutating matplotlib rcParams.

`evo.tools.plot` imports call `sns.set(style='darkgrid', ...)` at module
load, which mutates keys the paper style depends on. `_evo_boot` flips
`SETTINGS.plot_seaborn_enabled = False` before evo loads, so the
mutation should be neutered. This test asserts that the state after
`apply_paper_style()` survives transitive evo import.

Run:
    cd pymantaray/localization && uv run pytest tests/test_no_rcparam_pollution.py
"""

from __future__ import annotations

import os
import sys

# Ensure the tests can import modules from the parent (localization) dir.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Keys seaborn's 'darkgrid' + 'notebook' context mutate. Sourced from
# seaborn/rcmod.py _style_keys + _context_keys (verified inline).
_SEABORN_MUTATED_KEYS = (
    # style keys
    "axes.facecolor", "axes.edgecolor", "axes.grid", "axes.axisbelow",
    "axes.labelcolor", "figure.facecolor", "grid.color", "grid.linestyle",
    "text.color", "xtick.color", "ytick.color",
    "xtick.direction", "ytick.direction",
    "xtick.bottom", "xtick.top", "ytick.left", "ytick.right",
    "axes.spines.left", "axes.spines.bottom",
    "axes.spines.right", "axes.spines.top",
    "font.family",
    # context keys
    "font.size", "axes.labelsize", "axes.titlesize",
    "xtick.labelsize", "ytick.labelsize",
    "legend.fontsize", "axes.linewidth", "grid.linewidth",
    "xtick.major.width", "ytick.major.width",
    "xtick.minor.width", "ytick.minor.width",
    "xtick.major.size", "ytick.major.size",
    "xtick.minor.size", "ytick.minor.size",
)


def test_paper_style_after_evo_matches_intent() -> None:
    """apply_paper_style() after transitive evo import must produce
    the paper-figure state.

    Paper plots call `apply_paper_style()` inside the plot function so
    that ordering-wise the sequence is:
        module-top imports (transitively load evo, which mutates rcParams)
        -> plot function starts
        -> apply_paper_style() re-pins every key we care about
        -> figure is drawn
    This test asserts that after that sequence the important keys
    (font.family, axes.edgecolor, xtick.bottom, ...) match paper_style's
    intent, i.e. the compensator list in paper_style.py is complete.
    """
    # Force the evo import chain to run BEFORE we apply paper_style.
    import _evo_boot  # noqa: F401 -- neutralize the seaborn.set branch first
    import evo.tools.plot  # noqa: F401
    import py_factor_graph.utils.plot_utils  # noqa: F401

    from paper_style import apply_paper_style
    apply_paper_style()

    # These are the specific compensators paper_style pins to fix
    # evo's mutations. If any regresses, the paper figure silently
    # drifts (font.family especially — the recent recurring pain).
    intent = {
        "font.family":    "serif",
        "axes.edgecolor": "black",
        "xtick.bottom":   True,
        "ytick.left":     True,
    }
    diffs = {
        k: (want, plt.rcParams[k])
        for k, want in intent.items()
        if not _matches(plt.rcParams[k], want)
    }
    assert not diffs, (
        f"apply_paper_style() failed to restore expected rcParams "
        f"after evo import: {diffs}. Update paper_style.py to pin "
        f"these keys, or move the apply_paper_style() call to after "
        f"the offending import."
    )


def _matches(actual, want) -> bool:
    """Handle matplotlib's list-wrapping of font.family."""
    if isinstance(actual, list):
        return want in actual
    return actual == want
