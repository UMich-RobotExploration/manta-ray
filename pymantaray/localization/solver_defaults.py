"""Shared SolverConfig builder so run_solver.py and the MC driver stay in sync.

Holds the noise/odom/prior constants that used to live inline in run_solver.py.
`build_default_config(fg_data, **overrides)` returns a `SolverConfig` with the
tuned defaults; `**overrides` replaces any named field.
"""

from copy import deepcopy

import numpy as np
from py_factor_graph.factor_graph import FactorGraphData

from pyfg_to_gtsam import SolverConfig, odom_cadence_from_fg


# Position prior stddev (meters) used for GPS, landmark, and depth-prior defaults.
DEFAULT_POS_PRIOR: float = 0.1

# Odom noise model (per-edge, GTSAM Pose3 tangent order [rx, ry, rz, tx, ty, tz]).
#
#   sigma_i = sqrt( (frac_i * |motion_i|)**2 + (drift_rate_i * cadence_dt)**2 )
#
# The first term is velocity-scale error on the recorded motion. The second
# is time-based INS drift — constant per edge at fixed cadence.
ANGULAR_NOISE: float = 1e-6   # velocity-scale fraction for rotation
XY_FRAC: float = 0.05          # 5% scale error on recorded xy motion
Z_FRAC: float = 0.001           # 0.1% scale error on recorded z motion

ODOM_DRIFT_RATE_TRANS: float = 0.0125   # m/s, translation INS drift rate
ODOM_DRIFT_RATE_ROT: float = 5e-7       # rad/s, rotation INS drift rate

# Factor-graph rotation belief (rad-per-rad); translation fractions inherit
# from odom_noise so the factor belief matches the injected scale error.
GTSAM_ROT_BELIEF: float = 1e-2

# GPS prior: position stddev from DEFAULT_POS_PRIOR; rotation effectively unconstrained.
GPS_ROT_SIGMA: float = 2.0

DEPTH_PRIOR_SIGMA: float = 0.01   # meters (1 cm) — pressure-sensor stddev

RANGE_NOISE_STDDEV: float = 1.0   # meters


def build_default_config(fg_data: FactorGraphData,
                         **overrides) -> SolverConfig:
    """Construct the project's default SolverConfig, then apply overrides.

    Use kwargs to swap fields without duplicating the tuning block at every
    call site — e.g. `build_default_config(fg, seed=7, use_true_ranges=True)`.

    Unknown kwargs raise TypeError from SolverConfig.__init__, so misspellings
    fail loudly.
    """
    odom_noise = np.array(
        [ANGULAR_NOISE, ANGULAR_NOISE, ANGULAR_NOISE,
         XY_FRAC, XY_FRAC, Z_FRAC],
        dtype=np.float64)

    # Factor belief: rotation pessimistic, translation matches injected scale.
    odom_gtsam_noise = deepcopy(odom_noise)
    odom_gtsam_noise[:3] = GTSAM_ROT_BELIEF

    gps_prior_sigmas = np.array(
        [GPS_ROT_SIGMA, GPS_ROT_SIGMA, GPS_ROT_SIGMA,
         DEFAULT_POS_PRIOR, DEFAULT_POS_PRIOR, DEFAULT_POS_PRIOR],
        dtype=np.float64)

    defaults = dict(
        odom_noise_sigmas=odom_noise,
        between_noise_sigmas=odom_gtsam_noise,
        range_noise_stddev=RANGE_NOISE_STDDEV,
        add_range_noise=True,
        include_ranges=True,
        landmark_prior_sigma=DEFAULT_POS_PRIOR,
        gps_prior_sigmas=gps_prior_sigmas,
        depth_prior_sigma=DEPTH_PRIOR_SIGMA,
        # "pose3" uses PriorFactorPose3 (stock C++, no pybind callback per
        # linearization). "custom" uses a Python CustomFactor which crosses
        # the pybind boundary every LM iteration -- fine on small graphs, but
        # meaningful overhead at 7k+ poses. The two modes constrain slightly
        # different quantities (world-z vs body-frame retract z), but are
        # identical when robot rotation is the identity, as it is in the
        # current-drift floats.
        depth_prior_mode="pose3",
        add_depth_noise=True,
        odom_cadence_dt=odom_cadence_from_fg(fg_data),
        odom_drift_rate_trans=ODOM_DRIFT_RATE_TRANS,
        odom_drift_rate_rot=ODOM_DRIFT_RATE_ROT,
    )
    defaults.update(overrides)
    return SolverConfig(**defaults)
