"""Central styling helper for paper figures.

Any plotter that produces a figure destined for the conference paper
should call ``apply_paper_style()`` before it constructs figures. This
switches matplotlib to the SciencePlots ``ieee`` theme and pins the
knobs seaborn would otherwise silently clobber on evo import.

Retune the whole paper by changing ``BASE_FONT_SIZE`` here; every
plotter picks it up automatically on next run.
"""

# side-effect import: neutralize evo.tools.plot's sns.set(darkgrid)
# BEFORE any plotter transitively imports evo. Must stay above every
# other import that can pull evo into the process.
import _evo_boot  # noqa: F401
import matplotlib.pyplot as plt  # noqa: E402
import scienceplots  # noqa: F401,E402


BASE_FONT_SIZE = 10


def apply_paper_style() -> None:
    """Activate SciencePlots' IEEE style with our paper overrides.

    ``no-latex`` variant avoids requiring a system LaTeX install;
    matplotlib mathtext handles ``$r_{\\mathrm{error}}$``-style labels
    cleanly.
    """
    plt.style.use(["science", "ieee", "no-latex"])
    plt.rcParams.update({
        "font.size":       BASE_FONT_SIZE,
        "font.family":     "serif",  # evo silently flips to sans-serif
        "axes.labelsize":  BASE_FONT_SIZE + 1,
        "axes.titlesize":  BASE_FONT_SIZE + 1,
        "xtick.labelsize": BASE_FONT_SIZE,
        "ytick.labelsize": BASE_FONT_SIZE,
        "legend.fontsize": BASE_FONT_SIZE - 1,
        "savefig.dpi":     400,
        "figure.dpi":      400,
        # Seaborn darkgrid compensators (verified against seaborn.rcmod
        # _style_keys / _context_keys). Only kept in case _evo_boot ever
        # fails silently -- if pollution is neutralized upstream these
        # are redundant but harmless.
        "axes.edgecolor":  "black",
        "axes.linewidth":  0.6,
        "xtick.bottom":    True,
        "ytick.left":      True,
    })
