# pymantaray/localization/ — Factor-graph solver

Reads the `.pfg` factor graph emitted by `mantaray_core`, assembles it
as a centralized GTSAM problem, solves it with Levenberg-Marquardt, and
reports absolute pose error against ground truth via
[`evo`](https://github.com/MichaelGrupp/evo). Produces every APE and
range-bias figure in the accompanying paper.

## Setup

```bash
cd pymantaray/localization
uv sync
```

Python 3.11. Independent from
[`pymantaray/acoustics/`](../acoustics/README.md) (Python 3.12).

## Usage

### Single-scenario solve

[`run_solver.py`](run_solver.py) defaults to the paper's Beaufort
reference factor graph, shipped in-tree at
`mantaray/results/arctic/beaufort-fleet-week/output.pfg`:

```bash
uv run python run_solver.py
```

Prints initial vs final graph error and writes trajectory + APE plots
next to the `.pfg`, through `WORK_DIR = os.path.dirname(FILE_PATH)`.
Switching to the shipped Fram Strait graph, or to any other
`output.pfg` the C++ side produced, is a one-line edit to `FILE_PATH`
at the top of the script.

### Monte Carlo (paper reproduction)

The paper's headline numbers come from 25 seeds × 2 range conditions
(refracted vs straight-line) per environment. Regenerating them is two
scripts:

```bash
# Runner: writes mc_paired_ape.npz next to the input .pfg.
# Edit FILE_PATH and SEEDS at the top of the file first.
uv run python mc_paired_ape.py

# Plotter: reads the .npz, writes mc_pooled_ape_dist.png (violins).
uv run python plot_mc_paired_ape.py
```

The paper's cross-environment comparison uses two runs of the pair,
one per env, then [`plot_mc_env_compare.py`](plot_mc_env_compare.py)
to overlay them.

## Reproducing paper figures

| Figure | Script | Input |
|---|---|---|
| Fig. 2B — factor graph 3D | [`plot_factor_graph_3d`](debug_factor_graph_ranges.py) in `debug_factor_graph_ranges.py` | `output.pfg` |
| Fig. 5 — fleet layout | [`plot_fleet_layout.py`](plot_fleet_layout.py) | `mantaray/sim_config/paper_beaufort_fleet_week_sim.json` |
| Fig. 6 — range bias | [`plot_range_bias_paper`](debug_factor_graph_ranges.py) in `debug_factor_graph_ranges.py` | `output.pfg` (both envs) |
| Fig. 7 — pooled APE | [`plot_mc_paired_ape.py`](plot_mc_paired_ape.py) | `mc_paired_ape.npz` |
| Fig. 8 — Monte Carlo trajectories | [`plot_mc_trajectories.py`](plot_mc_trajectories.py) | `mc_trajectories.npz` |

All paper-figure plotters use the shared [`paper_style.py`](paper_style.py)
helper (SciencePlots IEEE style, serif fonts, 400 dpi).

## Noise model

Odometry noise on each edge is a per-component composite:

    sigma_i = sqrt( (frac_i * |motion_i|)^2 + (drift_rate_i * dt)^2 )

The velocity-scale term `frac_i * |motion_i|` grows with per-edge
displacement (DVL-like); the drift term `drift_rate_i * dt` is a
constant per-edge time-drift standard deviation (INS-like) that
dominates when a drifter loiters near zero velocity. Range noise is
additive Gaussian with a scalar `sigma_r` (default 1 m). The tuning
constants live in [`solver_defaults.py`](solver_defaults.py).

For the pre-graph noise injection stage, ranges and depths are
re-perturbed per Monte Carlo seed; GNSS fixes carry the noise the C++
simulator already applied.

## Modules

| Module | Purpose |
|---|---|
| [`run_solver.py`](run_solver.py) | Single-scenario entry point |
| [`pyfg_to_gtsam.py`](pyfg_to_gtsam.py) | `.pfg` → GTSAM factor graph + Values (`FactorGraphSolver`, `SolverConfig`, `per_pose_ape`, `_odom_sigma_from_motion_and_drift`) |
| [`solver_defaults.py`](solver_defaults.py) | Noise + prior sigma constants; `build_default_config(fg)` |
| [`mc_paired_ape.py`](mc_paired_ape.py) | Monte Carlo paired-condition runner (produces `.npz`) |
| [`plot_mc_paired_ape.py`](plot_mc_paired_ape.py) | Paper Figure 7 violin plot |
| [`plot_mc_env_compare.py`](plot_mc_env_compare.py) | Cross-environment RMSE bar chart |
| [`plot_mc_trajectories.py`](plot_mc_trajectories.py) | Monte Carlo trajectory ensemble figure |
| [`plot_fleet_layout.py`](plot_fleet_layout.py) | Paper Figure 5 |
| [`visualize_solver.py`](visualize_solver.py) | evo APE plots, per-robot trajectories, hull-based coverage masking |
| [`debug_factor_graph_ranges.py`](debug_factor_graph_ranges.py) | Range-error diagnostics; produces paper Figures 2B and 6 |
| [`debug_factor_graphs.py`](debug_factor_graphs.py) | Post-solve factor / residual introspection (leverage, Cook's distance) |
| [`hull_utils.py`](hull_utils.py) | 2D convex hull of the landmark set for coverage masking |
| [`animate_trajectories.py`](animate_trajectories.py) | PyVista-based MP4 renderer for a solve (README-banner asset) |
| [`report_range_failures*.py`](report_range_failures.py) | LaTeX tables of ranging-drop rates by link category |
| [`paper_style.py`](paper_style.py) + [`_evo_boot.py`](_evo_boot.py) | Shared IEEE SciencePlots setup + boot shim that neutralizes evo's seaborn theme mutation |

## Testing

```bash
uv run python tests/test_no_rcparam_pollution.py
```

Asserts that after any evo import, `apply_paper_style()` restores every
rcParam the paper figures depend on. Guards against evo / seaborn /
SciencePlots version drift silently reintroducing style pollution.

## Conventions

- Scripts are not CLI-driven; edit path constants at the top of each
  file and re-run.
- Every paper plotter goes through [`paper_style.py`](paper_style.py);
  every non-paper plotter can render with matplotlib defaults.
- Diagnostic PNG writers are gated behind a `WRITE_DIAGNOSTICS = False`
  flag in the scripts that own them, so paper-figure regeneration
  doesn't pollute the output directory with per-robot bias plots and
  ATE delta traces.
