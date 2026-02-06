"""Reader functions for aubellhop.

This file provides a collection of reading methods for both input
and output Bellhop files.

Reading the environment file has substantial logic when relies on
the state of the environment information it is passed. Therefore, this is
managed by class `EnvironmentReader()`, used internally by the `Environment`
class via its class method `Environment.from_file()`.

Subsequent files (ssp, bty, ati, brc, trc) are read automatically, but public
interfaces are provided for these for the purposes of mix and matching data
if desired.

Finally, readers are also provides for output files for rays, arrivals, and
binary shd files. Again, these are used automatically by `aubellhop` after
executing a computation, but public methods are provides to help with
testing and verification.

"""

from __future__ import annotations

import os
from struct import unpack as _unpack
from pathlib import Path
from typing import Any, TextIO, cast
from numpy.typing import NDArray

import numpy as np
import pandas as pd
from aubellhop.constants import BHStrings, FlagMaps, FileExt, MiscDefaults
from aubellhop.environment import Environment


class EnvironmentReader:
    """Read and parse Bellhop environment files.

    Although this class is only used for one task,
    the use of a class provides the clearest code interface compared
    to nested functions, which either implicitly set
    dict parameters, or have many repeated and superfluous
    arguments as dicts are passed in and returned at
    each stage.
    """

    def __init__(self, env: Environment, fname: str):
        """Initialize reader with filename.

        Args:
            fname: Path to .env file (with or without extension)
        """
        self.env = env
        self.fname, self.fname_base = _prepare_filename(fname, FileExt.env, "Environment")

    def read(self) -> Environment:
        """Do the reading..."""
        with open(self.fname, 'r') as f:
            self._read_header(f)
            self._read_top_boundary(f)
            next_line = self._read_sound_speed_profile(f)
            self._read_bottom_boundary(f, next_line)
            self._read_sources_receivers_task(f)
            self._read_beams_limits(f)
            self._read_gaussian_params(f)
        return self.env

    def _read_header(self, f: TextIO) -> None:
        """Read environment file header"""
        self.env['name'] = self._unquote_string(_read_next_valid_line(f))
        self.env['frequency'] = _parse_vector(_read_next_valid_line(f))
        self.env["_num_media"] = _parse_line_int(_read_next_valid_line(f))

    def _read_top_boundary(self, f: TextIO) -> None:
        """Read environment file top boundary options (multiple lines)"""

        # Line 4: Top boundary options
        topopt_line = _read_next_valid_line(f)
        topopt = self._unquote_string(topopt_line) + "      "
        self.env["soundspeed_interp"]          = self._opt_lookup("Interpolation",          topopt[0], FlagMaps.soundspeed_interp)
        self.env["surface_boundary_condition"] = self._opt_lookup("Top boundary condition", topopt[1], FlagMaps.surface_boundary_condition)
        self.env["attenuation_units"]          = self._opt_lookup("Attenuation units",      topopt[2], FlagMaps.attenuation_units)
        self.env["volume_attenuation"]         = self._opt_lookup("Volume attenuation",     topopt[3], FlagMaps.volume_attenuation)
        self.env["_altimetry"]                 = self._opt_lookup("Altimetry",              topopt[4], FlagMaps._altimetry)
        self.env["_single_beam"]               = self._opt_lookup("Single beam",            topopt[5], FlagMaps._single_beam)
        if self.env["_altimetry"] == BHStrings.from_file:
            self.env["surface"], self.env["surface_interp"] = read_ati(self.fname_base)

        if self.env["volume_attenuation"] == BHStrings.francois_garrison:
            fg_spec_line = _read_next_valid_line(f)
            fg_parts = _parse_line(fg_spec_line)
            self.env["_fg_salinity"]    = _float(fg_parts[0])
            self.env["_fg_temperature"] = _float(fg_parts[1])
            self.env["_fg_pH"]          = _float(fg_parts[2])
            self.env["_fg_depth"]       = _float(fg_parts[3])

        # Line 4a: Boundary condition params
        if self.env["surface_boundary_condition"] == BHStrings.acousto_elastic:
            surface_props_line = _read_next_valid_line(f)
            surface_props = _parse_line(surface_props_line, none_pad=6)
            self.env['_surface_min']               = _float(surface_props[0])
            self.env['surface_soundspeed']         = _float(surface_props[1])
            self.env['_surface_soundspeed_shear']  = _float(surface_props[2])
            self.env['surface_density']            = _float(surface_props[3], scale=1000)  # convert from g/cm³ to kg/m³
            self.env['surface_attenuation']        = _float(surface_props[4])
            self.env['_surface_attenuation_shear'] = _float(surface_props[5])

        # Line 4b: Biological layer properties
        if self.env["volume_attenuation"] == BHStrings.biological:
            self.env['biological_layer_parameters'] = self._read_biological_layers(f)

    def _read_biological_layers(self, f: TextIO) -> pd.DataFrame:
        """Read biological layer parameters for attenuation due to fish."""
        next_line = _read_next_valid_line(f)
        npoints = int(next_line)
        z1 = []
        z2 = []
        f0 = []
        QQ = []
        a0 = []
        for i in range(npoints):
            line = _read_next_valid_line(f)
            parts = _parse_line(line)
            if len(parts) == 5:
                z1.append(_float(parts[0]))
                z2.append(_float(parts[1]))
                f0.append(_float(parts[2]))
                QQ.append(_float(parts[3]))
                a0.append(_float(parts[4]))
        if len(z1) != npoints:
            raise ValueError(f"Expected {npoints} points, but found {len(z1)}")
        return pd.DataFrame({"z1": z1, "z2": z2, "f0": f0, "Q": QQ, "a0": a0})

    def _read_sound_speed_profile(self, f: TextIO) -> str:
        """Read environment file sound speed profile"""

        # SSP depth specification
        ssp_spec_line = _read_next_valid_line(f)
        ssp_spec_line = ssp_spec_line.replace(",", " ")
        ssp_parts = _parse_line(ssp_spec_line, none_pad=3)
        self.env['_mesh_npts']   = _int(ssp_parts[0])
        self.env['_depth_sigma'] = _float(ssp_parts[1])
        self.env['depth_max']    = _float(ssp_parts[2])
        self.env['depth'] = self.env['depth_max']

        # Read SSP points and from file if applicable
        ssp_lines, next_line = self._read_until_quote(f)
        self.env['soundspeed'] = self._read_ssp_points(ssp_lines)
        if self.env["soundspeed_interp"] == BHStrings.quadrilateral:
            self.env['soundspeed'] = read_ssp(self.fname_base, self.env['soundspeed'].index)
        elif self.env["soundspeed_interp"] == BHStrings.hexahedral:
            self.env['soundspeed'] = read_ssp_3d(self.fname_base)
        return next_line

    def _read_until_quote(self, f: TextIO) -> tuple[list[str],str]:
        """Read lines until one starts with ' character."""
        lines: list[str] = []
        while True:
            line = f.readline()
            if not line:
                raise EOFError("File ended during env file reading of SSP points.")
            line = line.strip()
            if not line: # completely empty line
                continue
            if line.startswith("'"): # Check if this is a bottom boundary line (starts with quote)
                return lines, line
            lines.append(line)

    def _read_ssp_points(self, lines: list[str]) -> pd.DataFrame:
        """Read sound speed profile points until we find the bottom boundary line

           Default values are according to 'EnvironmentalFile.htm'."""

        ssp_depth: list[float] = []
        ssp_speed: list[float] = []
        ssp_shear: list[float] = []
        ssp_density: list[float] = []
        ssp_atten: list[float] = []
        ssp_att_shear: list[float] = []
        ssp = dict(depth=0.0, speed=MiscDefaults.sound_speed, speed_shear=0.0, density=MiscDefaults.density, atten=0.0, att_shear=0.0)

        MAX_SSP_COLS: int = 6
        sound_speed_param_count: int = 0
        for line in lines:
            raw_parts = _parse_line(line)
            parts = (raw_parts + [None] * MAX_SSP_COLS)[:MAX_SSP_COLS]
            if parts[0] is None: # empty line after stripping comments
                continue
            sound_speed_param_count = max(len(raw_parts), sound_speed_param_count)
            for k, v in zip(ssp, parts):
                if v is not None:
                    ssp[k] = cast(float, _float(v)) # "fill in the blanks" if empty entries on any lines
            ssp_depth.append(ssp["depth"])
            ssp_speed.append(ssp["speed"])
            ssp_shear.append(ssp["speed_shear"])
            ssp_density.append(ssp["density"] * 1000.0) # units scaling
            ssp_atten.append(ssp["atten"])
            ssp_att_shear.append(ssp["att_shear"])

        if len(ssp_speed) == 0:
            raise ValueError("No SSP points were found in the env file.")
        elif len(ssp_speed) == 1:
            raise ValueError("Only one SSP point found but at least two required (top and bottom)")

        df = pd.DataFrame({
                "speed": ssp_speed,
                "shear_speed": ssp_shear,
                "density": ssp_density,
                "attenuation": ssp_atten,
                "shear_attenuation": ssp_att_shear
            }, index=pd.Index(ssp_depth))
        df = df.iloc[:, :(sound_speed_param_count-1)]  # Keep only the max number of columns read
        df.index.name = "depth"
        return df

    def _read_bottom_boundary(self, f: TextIO, bottom_line: str) -> None:
        """Read environment file bottom boundary condition"""
        bottom_parts = _parse_line(bottom_line,none_pad=3)
        botopt = self._unquote_string(cast(str,bottom_parts[0])) + "  " # cast() => I promise this is a str :)
        self.env["bottom_boundary_condition"] = self._opt_lookup("Bottom boundary condition", botopt[0], FlagMaps.bottom_boundary_condition)
        self.env["_bathymetry"]               = self._opt_lookup("Bathymetry",                botopt[1], FlagMaps._bathymetry)
        self.env['bottom_roughness']       = _float(bottom_parts[1])
        self.env['bottom_beta']            = _float(bottom_parts[2])
        self.env['bottom_transition_freq'] = _float(bottom_parts[3])
        if self.env["_bathymetry"] == BHStrings.from_file:
            self.env["depth"], self.env["bottom_interp"] = read_bty(self.fname_base)

        # Bottom properties (depth, sound_speed, density, absorption)
        if self.env["bottom_boundary_condition"] == BHStrings.acousto_elastic:
            bottom_props_line = _read_next_valid_line(f)
            bottom_props = _parse_line(bottom_props_line,none_pad=6)
            self.env['_bottom_depth'] = _float(bottom_props[0])
            self.env['bottom_soundspeed'] = _float(bottom_props[1])
            self.env['_bottom_soundspeed_shear'] = _float(bottom_props[2])
            self.env['bottom_density'] = _float(bottom_props[3], 1000)  # convert from g/cm³ to kg/m³
            self.env['bottom_attenuation'] = _float(bottom_props[4])
            self.env['_bottom_attenuation_shear'] = _float(bottom_props[5])
        elif self.env["bottom_boundary_condition"] == BHStrings.grain:
            bottom_props_line = _read_next_valid_line(f)
            bottom_props = _parse_line(bottom_props_line, none_pad=6)
            self.env['_bottom_depth'] = _float(bottom_props[0])
            self.env['bottom_grain_size'] = _float(bottom_props[1])

    def _read_sources_receivers_task(self, f: TextIO) -> None:
        """Read environment file sources, receivers, and task.

        Bellhop and Bellhop3D have different numbers of variables specified before
        the task line. Luckily we can detect that reliably by looking for a line which
        starts with `'` to mark the definition of the tasks."""

        sr_lines = []
        while True:
            next_line = _read_next_valid_line(f)
            if next_line.startswith("'"):
                break
            sr_lines.append(next_line)

        self._parse_src_rcv(sr_lines)
        self._parse_task(next_line)

    def _parse_src_rcv(self, sr_lines: list[str]) -> None:
        """Parse the N lines read defining sources and receivers and assign the corresponding variables."""
        nlines = len(sr_lines)
        if nlines == 3: # sometimes see a shorthand version like: ['1   25.0 /', '10 100.0 1000.0 /', '1,  80.0']
            self.env['_dimension'] = 2
            val0 = np.asarray(_parse_vector(sr_lines[0]))
            val1 = np.asarray(_parse_vector(sr_lines[1]))
            val2 = np.asarray(_parse_vector(sr_lines[2]))
            self.env['source_ndepth']   = int(val0[0])
            self.env['receiver_nrange'] = int(val1[0])
            self.env['receiver_ndepth'] = int(val2[0])
            self.env['source_depth']    = val0[1:]
            self.env['receiver_depth']  = val1[1:]
            self.env['receiver_range']  = val2[1:] * 1000.0 # convert km to m
        elif nlines == 6:
            self.env['_dimension'] = 2
            self.env['source_ndepth']   = _parse_line_int(sr_lines[0])
            self.env['receiver_ndepth'] = _parse_line_int(sr_lines[2])
            self.env['receiver_nrange'] = _parse_line_int(sr_lines[4])
            self.env['source_depth']    = _parse_vector(sr_lines[1])
            self.env['receiver_depth']  = _parse_vector(sr_lines[3])
            self.env['receiver_range']  = _parse_vector(sr_lines[5]) * 1000.0 # convert km to m
        elif nlines == 12:
            self.env['_dimension'] = 3
            self.env['source_nrange']      = _parse_line_int(sr_lines[0])
            self.env['source_ncrossrange'] = _parse_line_int(sr_lines[2])
            self.env['source_ndepth']      = _parse_line_int(sr_lines[4])
            self.env['receiver_ndepth']    = _parse_line_int(sr_lines[6])
            self.env['receiver_nrange']    = _parse_line_int(sr_lines[8])
            self.env['receiver_nbearing']  = _parse_line_int(sr_lines[10])
            self.env['source_range']       = _parse_vector(sr_lines[1]) * 1000.0 # convert km to m
            self.env['source_cross_range'] = _parse_vector(sr_lines[3]) * 1000.0 # convert km to m
            self.env['source_depth']       = _parse_vector(sr_lines[5])
            self.env['receiver_depth']     = _parse_vector(sr_lines[7])
            self.env['receiver_range']     = _parse_vector(sr_lines[9]) * 1000.0 # convert km to m
            self.env['receiver_bearing']   = _parse_vector(sr_lines[11])
        else:
            print("SCANNED SRC/RCV LINES:")
            print(sr_lines)
            raise RuntimeError(
                "The python parsing of Bellhop's so-called 'list-directed IO' is not robust."
                f"Expected to read 6 or 12 lines (2D or 3D cases); found: {nlines}")

    def _parse_task(self, task_line: str) -> None:
        """Parse the 'task' line."""
        task_code = self._unquote_string(task_line) + "     "
        self.env['task']        = FlagMaps.task.get(task_code[0])
        self.env['beam_type']   = FlagMaps.beam_type.get(task_code[1])
        self.env['_sbp_file']   = FlagMaps._sbp_file.get(task_code[2])
        self.env['source_type'] = FlagMaps.source_type.get(task_code[3])
        self.env['grid_type']   = FlagMaps.grid_type.get(task_code[4])
        if self.env['_dimension'] == 2:
            self.env['dimension'] = BHStrings.two_d
        else:
            self.env['dimension'] = FlagMaps.dimension.get(task_code[5])

        if self.env["_sbp_file"] == BHStrings.from_file:
            self.env["source_directionality"] = read_sbp(self.fname_base)

    def _read_beams_limits(self, f: TextIO) -> None:
        """Read environment file beams and limits"""

        # Number of beams
        beam_num_line = _read_next_valid_line(f)
        beam_num_parts = _parse_line(beam_num_line, none_pad=1)
        self.env['beam_num'] = int(beam_num_parts[0] or 0)
        if self.env["_single_beam"]: # defensive in case there is a spurious value in here
            self.env['single_beam_index'] = _int(beam_num_parts[1])

        # Beam angles (beam_angle_min, beam_angle_max)
        angles_line = _read_next_valid_line(f)
        angle_parts = _parse_line(angles_line, none_pad=2)
        self.env['beam_angle_min'] = _float(angle_parts[0])
        self.env['beam_angle_max'] = _float(angle_parts[1])

        if self.env['_dimension'] == 3:
            # Beam bearing fan
            beam_num_line = _read_next_valid_line(f)
            beam_num_parts = _parse_line(beam_num_line, none_pad=1)
            self.env['beam_angle_num'] = int(beam_num_parts[0] or 0)
            angles_line = _read_next_valid_line(f)
            angle_parts = _parse_line(angles_line, none_pad=2)
            self.env['beam_bearing_min'] = _float(angle_parts[0])
            self.env['beam_bearing_max'] = _float(angle_parts[1])

        # Ray tracing limits (step, max_depth, max_range) - last line
        limits_line = _read_next_valid_line(f)
        limits_parts = _parse_line(limits_line)
        self.env['step_size'] = _float(limits_parts[0])
        if self.env['_dimension'] == 2:
            self.env['simulation_depth'] = _float(limits_parts[1])
            self.env['simulation_range'] = _float(limits_parts[2], 1000.0)  # convert km to m
        else:
            self.env['simulation_range'] = _float(limits_parts[1], 1000.0)  # convert km to m
            self.env['simulation_cross_range'] = _float(limits_parts[2], 1000.0)  # convert km to m
            self.env['simulation_depth'] = _float(limits_parts[3])

    def _read_gaussian_params(self, f: TextIO) -> None:
        """Read parameters for Cerveny Gaussian Beams, if applicable"""
        if self.env['beam_type'] not in (BHStrings.gaussian_simple, BHStrings.ray):
            return None
        line = _read_next_valid_line(f)
        parts = _parse_line(line, none_pad=3)
        assert isinstance(parts[0],str)
        self.env['beam_width_type'] = self._unquote_string(parts[0])
        self.env['beam_epsilon_multipler'] = _float(parts[1])
        self.env['beam_range_loop'] = _float(parts[2],1000)

        line = _read_next_valid_line(f)
        parts = _parse_line(line, none_pad=3)
        self.env['beam_images_num'] = _int(parts[0])
        self.env['beam_window'] = _int(parts[1])
        self.env['beam_component'] = self._unquote_string(parts[2]) if parts[2] is not None else " "

    def _opt_lookup(self, name: str, opt: str, _map: dict[str, BHStrings]) -> str | None:
        opt_str = _map.get(opt)
        if opt_str is None:
            raise ValueError(f"{name} option {opt!r} not available")
        return opt_str

    def _unquote_string(self, line: str) -> str:
        """Extract string from within single quotes, possibly with commas too."""
        return line.strip().strip(",'")

########################################

def read_ssp(fname: str,
             depths: list[float] | NDArray[np.float64] | pd.DataFrame | None = None
            ) -> NDArray[np.float64] | pd.DataFrame:
    """Read a 2D sound speed profile (.ssp) file used by BELLHOP.

    This function reads BELLHOP's .ssp files which contain range-dependent
    sound speed profiles. The file format is:
    - Line 1: Number of range profiles (NPROFILES)
    - Line 2: Range coordinates in km (space-separated)
    - Line 3+: Sound speed values, one line per depth point across all ranges

    Parameters
    ----------
    fname : str
        Path to .ssp file (with or without .ssp extension)

    Returns
    -------
    numpy.ndarray or pandas.DataFrame
        For single-profile files: numpy array with [depth, soundspeed] pairs;
        for multi-profile files: pandas DataFrame with range-dependent sound speed data

    Notes
    -----
    **Return format:**

    - **Single-profile files (1 range)**: Returns a 2D numpy array with [depth, soundspeed] pairs,
      compatible with the `Environment()` soundspeed parameter.

    - **Multi-profile files (>1 ranges)**: Returns a pandas DataFrame where:

      - **Columns**: Range coordinates (in meters, converted from km in file)
      - **Index**: Depth indices (0, 1, 2, ... for each depth level in the file)
      - **Values**: Sound speeds (m/s)

      This DataFrame can be directly assigned to the `Environment()` soundspeed parameter
      for range-dependent acoustic modeling.

    **Note on depths**: For multi-profile files, depth indices are used (0, 1, 2, ...)
    since the actual depth coordinates come from the associated BELLHOP .env file.
    Users can modify the DataFrame index if actual depth values are known.

    Examples
    --------
    >>> import aubellhop as bh
    >>> # Single-profile file
    >>> ssp1 = bh.read_ssp("single_profile.ssp")  # Returns numpy array
    >>> env = bh.Environment()
    >>> env["soundspeed"] = ssp1
    >>>
    >>> # Multi-profile file
    >>> ssp2 = bh.read_ssp("tests/MunkB_geo_rot/MunkB_geo_rot.ssp")  # Returns DataFrame
    >>> env = bh.Environment()
    >>> env["soundspeed"] = ssp2  # Range-dependent sound speed

    **File format example:**

    ::

        30
        -50 -5 -1 -.8 -.75 -.6 -.4 -.2 0 0.2 0.4 0.6 0.8 1.0 1.2 1.4 1.6 1.8 2.0 2.2 2.4 2.6 2.8 3.0 3.2 3.4 3.6 3.8 4.0 10.0
        1500 1500 1548.52 1530.29 1526.69 1517.78 1509.49 1504.30 1501.38 1500.14 1500.12 1501.02 1502.57 1504.62 1507.02 1509.69 1512.55 1515.56 1518.67 1521.85 1525.10 1528.38 1531.70 1535.04 1538.39 1541.76 1545.14 1548.52 1551.91 1551.91
        1500 1500 1548.52 1530.29 1526.69 1517.78 1509.49 1504.30 1501.38 1500.14 1500.12 1501.02 1502.57 1504.62 1507.02 1509.69 1512.55 1515.56 1518.67 1521.85 1525.10 1528.38 1531.70 1535.04 1538.39 1541.76 1545.14 1548.52 1551.91 1551.91
    """

    fname, _ = _prepare_filename(fname, FileExt.ssp, "SSP")
    with open(fname, 'r') as f:
        nranges = int(_read_next_valid_line(f))
        range_line = _read_next_valid_line(f)
        ranges = np.array([_float(x) for x in _parse_line(range_line)])
        ranges_m = ranges * 1000 # Convert ranges from km to meters (as expected by Environment())

        if len(ranges) != nranges:
            raise ValueError(f"Expected {nranges} ranges, but found {len(ranges)}")

        # Read sound speed data - read all remaining lines as a matrix
        ssp_data = []
        line_num = 0
        for line in f:
            line_num += 1
            line = line.replace(","," ").strip()
            if line:  # Skip empty lines
                values = [_float(x) for x in line.split()]
                if len(values) != nranges:
                    raise ValueError(f"SSP line {line_num} has {len(values)} range values, expected {nranges}")
                ssp_data.append(values)

        ssp_array = np.array(ssp_data)
        ndepths = ssp_array.shape[0]

        # Create depth indices (actual depths would normally come from associated .env file)
        if depths is None:
            depths = np.arange(ndepths, dtype=float)

        if ndepths == 0 or len(depths) != ndepths:
            raise ValueError("Wrong number of depths found in sound speed data file"
                             f" (expected {ndepths}, found {ssp_array.shape[0]})")

        df = pd.DataFrame(ssp_array, index=pd.Index(depths), columns=ranges_m)
        df.index.name = "depth"
        return df

def read_ssp_3d(fname: str) -> dict[str, np.ndarray]:
    """Read a 3D sound speed profile (.ssp) file used by BELLHOP3D and return a 3D Numpy array.

    This function reads BELLHOP's .ssp files which contain range-dependent
    sound speed profiles. The file format is:
    - Line 1: Nx: Number of coordinates in the range (x) direction
    - Line 2: Range coordinates in km (space-separated)
    - Line 3: Ny: Number of coordinates in the crossrange (y) direction
    - Line 4: Crossrange coordinates in km (space-separated)
    - Line 5: Nz: Number of coordinates in the depth (z) direction
    - Line 6: Depth coordinates in km (space-separated)
    - Line 7+: Sound speed values, one line per crossrange and depth across all ranges
               (i.e., each line has Nx number of values and there are Ny*Nz lines)
    """

    fname, _ = _prepare_filename(fname, FileExt.ssp, "SSP")
    with open(fname, 'r') as f:

        nranges = int(_read_next_valid_line(f))
        range_line = _read_next_valid_line(f)
        ncrossranges = int(_read_next_valid_line(f))
        crossrange_line = _read_next_valid_line(f)
        ndepths = int(_read_next_valid_line(f))
        depth_line = _read_next_valid_line(f)

        ranges = np.array([_float(x) for x in _parse_line(range_line)])
        crossranges = np.array([_float(x) for x in _parse_line(crossrange_line)])
        depths = np.array([_float(x) for x in _parse_line(depth_line)])

        # Convert ranges from km to meters (as expected by Environment())
        ranges_m = ranges * 1000
        crossranges_m = crossranges * 1000
        depths_m = depths * 1000

        if len(ranges) != nranges:
            raise ValueError(f"Expected {nranges} ranges, but found {len(ranges)}")
        if len(crossranges) != ncrossranges:
            raise ValueError(f"Expected {ncrossranges} ranges, but found {len(crossranges)}")
        if len(depths) != ndepths:
            raise ValueError(f"Expected {ndepths} ranges, but found {len(depths)}")

        # Read sound speed data - read all remaining lines as a matrix
        ssp_data = []
        line_num = 0
        for line in f:
            line_num += 1
            line = line.replace(","," ").strip()
            if line:  # Skip empty lines
                values = [_float(x) for x in line.split()]
                if len(values) != nranges:
                    raise ValueError(f"SSP line {line_num} has {len(values)} range values, expected {nranges}")
                ssp_data.append(values)

        ssp_array = np.array(ssp_data)
        narray = ssp_array.shape[0]
        if narray != ncrossranges * ndepths:
            raise ValueError("Wrong number of entries found in sound speed data file"
                             f" (expected {ncrossranges} * {ndepths} = {ncrossranges * ndepths} lines, found {narray})")

        ssp_3d = ssp_array.reshape(ndepths, ncrossranges, nranges, order="C")
        ssp_xyz = np.transpose(ssp_3d, (2, 1, 0))

        return {"ssp": ssp_xyz, "ranges": ranges_m, "crossranges": crossranges_m, "depths": depths_m}

def read_ati(fname: str) -> tuple[NDArray[np.float64], str]:
    """Read an altimetry file used by Bellhop."""
    fname, _ = _prepare_filename(fname, FileExt.ati, "ATI")
    return _read_ati_bty(fname)

def read_bty(fname: str) -> tuple[NDArray[np.float64], str]:
    """Read a bathymetry file used by Bellhop."""
    fname, _ = _prepare_filename(fname, FileExt.bty, "BTY")
    return _read_ati_bty(fname)

def _read_ati_bty(fname: str) -> tuple[NDArray[np.float64], str]:
    """Read an altimetry (.ati) or bathymetry (.bty) file used by BELLHOP.

    This function reads BELLHOP's .bty files which define the bottom depth
    profile. The file format is:
    - Line 1: Interpolation type ('L' for linear, 'C' for curvilinear)
    - Line 2: Number of points
    - Line 3+: Range (km) and depth (m) pairs

    Parameters
    ----------
    fname : str
        Path to .ati/.bty file (with extension)

    Returns
    -------
    numpy.ndarray
        Numpy array with [range, depth] pairs compatible with Environment()

    Notes
    -----
    The returned array can be assigned to env["depth"] for range-dependent bathymetry.

    **Examples:**

    >>> import aubellhop as bh
    >>> bty,bty_interp = bh.read_bty("tests/MunkB_geo_rot/MunkB_geo_rot.bty")
    >>> env = bh.Environment()
    >>> env["depth"] = bty
    >>> env["depth_interp"] = bty_interp
    >>> arrivals = bh.calculate_arrivals(env)

    **File format example:**

    ::

        'L'
        5
        0 3000
        10 3000
        20 500
        30 3000
        100 3000
    """

    with open(fname, 'r') as f:
        # Read interpolation type ('L' or 'C')
        interp_type = _read_next_valid_line(f).strip("'\"")
        nvalues = 2
        if len(interp_type) > 1:
            nvalues = 7 if interp_type[1] == "L" else 2
            interp_type = interp_type[0]
        npoints = int(_read_next_valid_line(f))
        ranges = []
        depths = []
        wave_speed = []
        wave_attenuation = []
        density = []
        shear_speed = []
        shear_attenuation = []
        for i in range(npoints):
            try:
                line = _read_next_valid_line(f)
            except EOFError:
                break
            parts = _parse_line(line)
            if nvalues == 2 and len(parts) >= 2:
                ranges.append(_float(parts[0]))  # Range in km
                depths.append(_float(parts[1]))  # Depth in m
            elif nvalues == 7 and len(parts) == 7:
                ranges.append(_float(parts[0]))  # Range in km
                depths.append(_float(parts[1]))  # Depth in m
                wave_speed.append(_float(parts[2]))  #
                wave_attenuation.append(_float(parts[3]))  #
                density.append(_float(parts[4]))  #
                shear_speed.append(_float(parts[5]))  #
                shear_attenuation.append(_float(parts[6]))  #

        if len(ranges) != npoints:
            raise ValueError(f"Expected {npoints} altimetry/bathymetry points, but found {len(ranges)}")

        # Convert ranges from km to m for consistency with bellhop env structure
        ranges_m = np.array(ranges) * 1000
        depths_array = np.array(depths)
        if nvalues == 2:
            val_array = [ranges_m, depths_array]
        elif nvalues == 7:
            wave_speed_array = np.array(wave_speed)
            wave_attenuation_array = np.array(wave_attenuation)
            density_array = np.array(density)
            shear_speed_array = np.array(shear_speed)
            shear_attenuation_array = np.array(shear_attenuation)
            val_array = [
                         ranges_m,
                         depths_array,
                         wave_speed_array,
                         wave_attenuation_array,
                         density_array,
                         shear_speed_array,
                         shear_attenuation_array,
                        ]
        return np.column_stack(val_array), FlagMaps.depth_interp[interp_type]

def read_ati_3d(fname: str) -> dict[str, np.ndarray]:
    """Read an altimetry file used by Bellhop."""
    fname, _ = _prepare_filename(fname, FileExt.ati, "ATI")
    return _read_ati_bty3d(fname)

def read_bty_3d(fname: str) -> dict[str, np.ndarray]:
    """Read a bathymetry file used by Bellhop."""
    fname, _ = _prepare_filename(fname, FileExt.bty, "BTY")
    return _read_ati_bty3d(fname)

def _read_ati_bty3d(fname: str) -> dict[str, NDArray[np.float64]]:
    """Read an altimetry (.ati) or bathymetry (.bty) file used by BELLHOP.

    This function reads BELLHOP's .bty files which define the bottom depth
    profile. The file format is:
    - Line 1: Interpolation type ('R' always for Bellhop3D)
    - Line 2: Nx: Number of range points
    - Line 3: Range coordinates (x)
    - Line 4: Ny: Number of crossrange points
    - Line 5: Crossrange points (y)
    - Line 6+: 2D array of depths. Ny lines, one per crossrange, with Nx depths per line (space separated)

    Parameters
    ----------
    fname : str
        Path to .ati/.bty file (with extension)

    Returns
    -------
    dict
        Dictionary of ranges, crossranges, and 2D depth data

    Notes
    -----
    The returned array can be assigned to env["depth"] for range-dependent bathymetry.

    """

    with open(fname, 'r') as f:
        interp_type = _read_next_valid_line(f).strip("'\"")
        # if interp_type != "L":
        #     raise ValueError("3D depth files (.ati/.bty) can only have 'R' = regular grid as the first line")

        nranges = int(_read_next_valid_line(f))
        range_line = _read_next_valid_line(f)
        ncrossranges = int(_read_next_valid_line(f))
        crossrange_line = _read_next_valid_line(f)

        ranges = np.array([_float(x) for x in _parse_line(range_line)])
        crossranges = np.array([_float(x) for x in _parse_line(crossrange_line)])

        # Convert ranges from km to meters (as expected by Environment())
        ranges_m = ranges * 1000
        crossranges_m = crossranges * 1000

        depth_data = []
        line_num = 0
        for line in f:
            line_num += 1
            line = line.replace(","," ").strip()
            if line:  # Skip empty lines
                values = [_float(x) for x in line.split()]
                if len(values) != nranges:
                    raise ValueError(f"Depth line {line_num} has {len(values)} depth values, expected {nranges}")
                depth_data.append(values)

        depths = np.array(depth_data).transpose()
        if depths.shape[0] != nranges:
            raise ValueError("Wrong number of range entries found in depth file"
                             f" (expected {nranges}, found {depths.shape[0]})")
        if depths.shape[1] != ncrossranges:
            raise ValueError("Wrong number of crossrange entries found in depth file"
                             f" (expected {ncrossranges}, found {depths.shape[1]})")

    return {"depths": depths, "ranges": ranges_m, "crossranges": crossranges_m}


def read_sbp(fname: str) -> NDArray[np.float64]:
    """Read an source beam patterm (.sbp) file used by BELLHOP.

    The file format is:
    - Line 1: Number of points
    - Line 2+: Angle (deg) and power (dB) pairs

    Parameters
    ----------
    fname : str
        Path to .sbp file (with or without extension)

    Returns
    -------
    numpy.ndarray
        Numpy array with [angle, power] pairs
    """

    fname, _ = _prepare_filename(fname, FileExt.sbp, "SBP")
    with open(fname, 'r') as f:

        # Read number of points
        npoints = int(_read_next_valid_line(f))

        # Read range,depth pairs
        angles = []
        powers = []

        for i in range(npoints):
            try:
                line = _read_next_valid_line(f)
            except EOFError:
                break
            parts = _parse_line(line)
            assert isinstance(parts[0],str)
            assert isinstance(parts[1],str)
            if len(parts) >= 2:
                angles.append(float(parts[0]))  # Range in km
                powers.append(float(parts[1]))  # Depth in m

        if len(angles) != npoints:
            raise ValueError(f"Expected {npoints} points, but found {len(angles)}")

        # Return as [range, depth] pairs
        return np.column_stack([angles, powers])

def read_brc(fname: str) -> NDArray[np.float64]:
    """Read a BRC file and return array of reflection coefficients.


    This function reads BELLHOP's .brc files which define the reflection coefficient
    data. The file format is:
    - Line 1: Number of points
    - Line 2+: THETA(j)       RMAG(j)       RPHASE(j)

    Where:
    - THETA():  Angle (degrees)
    - RMAG():   Magnitude of reflection coefficient
    - RPHASE(): Phase of reflection coefficient (degrees)

    Parameters
    ----------
    fname : str
        Path to .brc/.trc file (extension required)

    Returns
    -------
    numpy.ndarray
        Numpy array with [theta, rmag, rphase] triplets compatible with Environment()

    Notes
    -----
    The returned array can be assigned to `env["bottom_reflection_coefficient"]`.
    The equivalent `read_trc()` data would be assigned to `env["surface_reflection_coefficient"]`.

    File format example
    -------------------
    ::

        3
        0.0   1.00  180.0
        45.0  0.95  175.0
        90.0  0.90  170.0

    Example
    -------
    >>> import aubellhop as bh
    >>> import aubellhop.readers as bhr
    >>> brc = bhr.read_refl_coeff("tests/MunkB_geo_rot/MunkB_geo_rot.brc")
    >>> env = bh.Environment()
    >>> env["bottom_reflection_coefficient"] = brc
    >>> arrivals = bh.calculate_arrivals(env)

    """
    fname, _ = _prepare_filename(fname, FileExt.brc, "BRC")
    return _read_refl_coeff(fname)

def read_trc(fname: str) -> NDArray[np.float64]:
    """Read a TRC file and return array of reflection coefficients.

    See equivalent `read_brc` function for documentation."""
    fname, _ = _prepare_filename(fname, FileExt.trc, "TRC")
    return _read_refl_coeff(fname)

def _read_refl_coeff(fname: str) -> NDArray[np.float64]:
    """Read a reflection coefficient (.brc/.trc) file used by BELLHOP."""

    with open(fname, 'r') as f:

        # Read number of points
        npoints = int(_read_next_valid_line(f))

        # Read range,depth pairs
        theta = []
        rmagn = []
        rphas = []

        for i in range(npoints):
            try:
                line = _read_next_valid_line(f)
            except EOFError:
                break
            parts = _parse_line(line)
            if len(parts) != 3:
                raise ValueError(f"Expected 3 reflection coefficient points, but found {len(parts)}")
            assert isinstance(parts[0],str)
            assert isinstance(parts[1],str)
            assert isinstance(parts[2],str)
            if len(parts) == 3:
                theta.append(float(parts[0]))
                rmagn.append(float(parts[1]))
                rphas.append(float(parts[2]))

        if len(theta) != npoints:
            raise ValueError(f"Expected {npoints} reflection coefficient points, but found {len(theta)}")

        # Return as [range, depth] pairs
        return np.column_stack([theta, rmagn, rphas])

def _read_next_valid_line(f: TextIO) -> str:
    """Read the next valid text line of an input file, discarding empty content.

    Args:
        f: File handle to read from

    Returns:
        Non-empty line with comments and whitespace removed

    Raises:
        EOFError: If end of file reached without finding valid content
    """
    while True:
        raw_line = f.readline()
        if not raw_line: # EOF
            raise EOFError("End of file reached before finding a valid line")
        line = raw_line.split('!', 1)[0].strip()
        if line:
            return line

def _parse_line(line: str, none_pad: int = 0) -> list[str | None]:
    """Parse a line, removing comments, /, and whitespace, and return the parts in a list"""
    line = line.split("!", 1)[0].split('/', 1)[0].strip()
    line = line.replace(","," ")
    return [*line.split(), *([None] * none_pad)]

def _parse_line_int(line: str) -> int | None:
    """Parse an integer on a line by itself. Strip spurious comma(s)."""
    parts = _parse_line(line)
    if parts[0] is None:
        return None
    return int(parts[0])

def _parse_vector(line: str) -> NDArray[np.float64] | float:
    """Parse a vector of floats with unknown number of values. Strip commas if necessary."""
    parts = _parse_line(line)
    val = [float(str(p).strip(",")) for p in parts]
    valout = np.array(val) if len(val) > 1 else val[0]
    return valout

def _float(x: str | None, scale: float = 1) -> float | None:
    """Permissive float-enator with unit scaling"""
    if x is None:
        return None
    return float(x.strip(",")) * scale

def _int(x: Any) -> int | None:
    """Permissive int-enator"""
    return None if x is None else int(x.strip(","))

def _prepare_filename(fname: str, ext: str, name: str) -> tuple[str,str]:
    """Checks filename is present and file exists."""
    if fname.endswith(ext):
        nchar = len(ext)
        fname_base = fname[:-nchar]
    else:
        fname_base = fname
        fname = fname + ext

    if not os.path.exists(fname):
        raise FileNotFoundError(f"{name} file not found: {fname}")

    return fname, fname_base

################################

def read_arrivals(fname: str) -> pd.DataFrame:
    """Read Bellhop arrivals file and parse data into a high level data structure"""
    reader = BellhopOutputReader(fname)
    return reader.read_arrivals()

def read_rays(fname: str) -> pd.DataFrame:
    """Read Bellhop rays file and parse data into a high level data structure"""
    reader = BellhopOutputReader(fname)
    return reader.read_rays()

def read_shd(fname: str) -> pd.DataFrame:
    """Read Bellhop shd file and parse data into a high level data structure"""
    reader = BellhopOutputReader(fname)
    return reader.read_shd()

class BellhopOutputReader:
    """Read and parse Bellhop output files."""

    def __init__(self, filename: str):
        """Initialize reader with filename.

        Args:
            filename: Path to file (with extension)
        """
        self.filename = filename
        self.filepath = self._ensure_file_exists(filename)

    def read_arrivals(self) -> pd.DataFrame:
        """Read Bellhop arrivals file and parse data into a high level data structure"""
        with self.filepath.open('rt') as f:
            hdr = f.readline()
            if hdr.find('2D') >= 0:
                freq = self._read_array(f, (float,))
                source_depth_info = self._read_array(f, (int,), float)
                source_depth_count = source_depth_info[0]
                source_depth = source_depth_info[1:]
                assert source_depth_count == len(source_depth)
                receiver_depth_info = self._read_array(f, (int,), float)
                receiver_depth_count = receiver_depth_info[0]
                receiver_depth = receiver_depth_info[1:]
                assert receiver_depth_count == len(receiver_depth)
                receiver_range_info = self._read_array(f, (int,), float)
                receiver_range_count = receiver_range_info[0]
                receiver_range = receiver_range_info[1:]
                assert receiver_range_count == len(receiver_range)
    #             else: # worry about 3D later
    #                 freq, source_depth_count, receiver_depth_count, receiver_range_count = _read_array(hdr, (float, int, int, int))
    #                 source_depth = _read_array(f, (float,)*source_depth_count)
    #                 receiver_depth = _read_array(f, (float,)*receiver_depth_count)
    #                 receiver_range = _read_array(f, (float,)*receiver_range_count)
            arrivals: list[pd.DataFrame] = []
            for j in range(source_depth_count):
                f.readline()
                for k in range(receiver_depth_count):
                    for m in range(receiver_range_count):
                        count = int(f.readline())
                        for n in range(count):
                            data = self._read_array(f, (float, float, float, float, float, float, int, int))
                            arrivals.append(pd.DataFrame({
                                'source_depth_ndx': [j],
                                'receiver_depth_ndx': [k],
                                'receiver_range_ndx': [m],
                                'source_depth': [source_depth[j]],
                                'receiver_depth': [receiver_depth[k]],
                                'receiver_range': [receiver_range[m]],
                                'arrival_number': [n],
                                # 'arrival_amplitude': [data[0]*np.exp(1j * data[1]* np.pi/180)],
                                'arrival_amplitude': [data[0] * np.exp( -1j * (np.deg2rad(data[1]) + freq[0] * 2 * np.pi * (data[3] * 1j +  data[2])))],
                                'time_of_arrival': [data[2]],
                                'complex_time_of_arrival': [data[2] + 1j*data[3]],
                                'angle_of_departure': [data[4]],
                                'angle_of_arrival': [data[5]],
                                'surface_bounces': [data[6]],
                                'bottom_bounces': [data[7]]
                            }, index=pd.Index([len(arrivals)+1])))
        return pd.concat(arrivals)

    def read_shd(self) -> pd.DataFrame:
        """Read Bellhop shd file and parse data into a high level data structure"""
        with self.filepath.open('rb') as f:
            recl, = _unpack('i', f.read(4))
            # _title = str(f.read(80))
            f.seek(4*recl, 0)
            ptype = f.read(10).decode('utf8').strip()
            assert ptype == 'rectilin', f'Invalid file format (expecting {ptype} == "rectilin")'
            f.seek(8*recl, 0)
            nfreq, ntheta, nsx, nsy, nsd, nrd, nrr, atten = _unpack('iiiiiiif', f.read(32))
            assert nfreq == 1, 'Invalid file format (expecting nfreq == 1)'
            assert ntheta == 1, 'Invalid file format (expecting ntheta == 1)'
            assert nsd == 1, 'Invalid file format (expecting nsd == 1)'
            f.seek(32*recl, 0)
            pos_r_depth = _unpack('f'*nrd, f.read(4*nrd))
            f.seek(36*recl, 0)
            pos_r_range = _unpack('f'*nrr, f.read(4*nrr))
            pressure = np.zeros((nrd, nrr), dtype=np.complex128)
            for ird in range(nrd):
                recnum = 10 + ird
                f.seek(recnum*4*recl, 0)
                temp = np.array(_unpack('f'*2*nrr, f.read(2*nrr*4)))
                pressure[ird,:] = temp[::2] + 1j*temp[1::2]
        return pd.DataFrame(pressure, index=pd.Index(pos_r_depth), columns=pd.Index(pos_r_range))

    def read_rays(self) -> pd.DataFrame:
        """Read Bellhop rays file and parse data into a high level data structure"""
        with self.filepath.open('rt') as f:
            hdr = f.readline()
            if hdr.find('BELLHOP-') >= 0:
                _dim = 2
            elif hdr.find('BELLHOP3D-') >= 0:
                _dim = 3
            else:
                # TODO: Tim override
                _dim = 3
            f.readline() # freq
            f.readline() # 1  1 1
            f.readline() # 50 50
            f.readline() # 0.0
            f.readline() # 25.0
            f.readline() # 'xyz'
            rays = []
            while True:
                s = f.readline()
                if s is None or len(s.strip()) == 0:
                    break
                a = float(s)
                pts, sb, bb = self._read_array(f, (int, int, int))
                ray = np.empty((pts, _dim))
                for k in range(pts):
                    ray[k,:] = self._read_array(f, (float,))
                rays.append(pd.DataFrame({
                    'angle_of_departure': [a],
                    'surface_bounces': [sb],
                    'bottom_bounces': [bb],
                    'ray': [ray]
                }))
        return pd.concat(rays)

    def _ensure_file_exists(self, filename: str) -> Path:
        path = Path(filename)
        if not path.exists():
            raise RuntimeError(f"Bellhop did not generate expected output file: {path}")
        return path

    def _read_array(self, f: TextIO, types: tuple[Any, ...], dtype: type = str) -> tuple[Any, ...]:
        """Wrapper around readline() to read in a 1D array of data"""
        p = f.readline().split()
        for j in range(len(p)):
            if len(types) > j:
                p[j] = types[j](p[j])
            else:
                p[j] = dtype(p[j])
        return tuple(p)
