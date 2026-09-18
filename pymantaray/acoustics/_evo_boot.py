"""Neutralize evo's module-load seaborn theme mutation.

`evo.tools.plot` runs `sns.set(style='darkgrid', ...)` unconditionally
at module load, which clobbers rcParams that our paper style depends on
(font.family, axes.edgecolor, xtick.bottom, etc.). This module flips
evo's own kill switch BEFORE evo.tools.plot is imported anywhere in the
process, so the seaborn call is skipped and no rcParams are touched.

Every module that transitively imports evo (directly or via
py_factor_graph.utils.plot_utils) should be reachable only through a
code path that imports `paper_style` first -- `paper_style` imports this
module, which sets the flag as an import-time side effect.

If evo ever renames the flag, the assert fires loudly instead of
silently regressing. If evo isn't installed (acoustics subproject),
the try/except turns this file into a no-op so the same paper_style.py
works verbatim across both subprojects.
"""

try:
    from evo.tools.settings import SETTINGS
except ImportError:
    pass
else:
    assert hasattr(SETTINGS, "plot_seaborn_enabled"), (
        "evo removed plot_seaborn_enabled; audit paper_style workarounds"
    )
    SETTINGS.plot_seaborn_enabled = False
