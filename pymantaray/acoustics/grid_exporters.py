"""Numpy serializers for the C++ Grid2D / Grid3D / GridVec consumers.

The C++ side (mantaray) loads bathymetry, sound speed, and currents from `.npy`
files arranged on a regular x/y (and optional z) grid. The exporters here own
the contract between the xarray side and that C++ layout:

- Bathymetry → Grid2D, flat layout `ix * ny + iy` (x-major / lon-major)
- Sound speed → Grid3D, flat layout `ix * ny*nz + iy * nz + iz`
- Currents (u, v) → GridVec, same layout as Grid3D

Each exporter validates its inputs *before* writing so misconfigured datasets
fail loudly here instead of producing silently-wrong files that crash the C++
solver later. Add new validation here, not at the call site.
"""

import os

import numpy as np
import xarray as xr


# Dim contracts for each exporter. The xarray DataArray we receive must have
# exactly these dim names — if it doesn't, the producer (e.g. oceanbench_data)
# forgot to assign coordinates correctly and the .transpose() call below would
# either fail with a confusing xarray error or silently scramble axes.
_BATHY_DIMS = ("lat", "lon")
_FIELD_3D_DIMS = ("lat", "lon", "depth")


def _validate_dims(da: xr.DataArray, expected: tuple[str, ...], func_name: str) -> None:
    """Raise if `da` doesn't have exactly the expected dim names (any order)."""
    if set(da.dims) != set(expected):
        raise ValueError(
            f"{func_name}: expected DataArray dims {set(expected)}, got {da.dims}. "
            f"Check that the upstream dataset still uses ('lat', 'lon'[, 'depth']) "
            f"as dim names — renaming or transposing upstream will silently break "
            f"the C++ Grid layout."
        )


def _validate_xy_coords(ds: xr.Dataset, func_name: str) -> None:
    """Raise if x_m / y_m projected coords are missing or misaligned with lon/lat.

    The projected coordinates are produced by the local-projection step in the
    caller and must align 1:1 with the `lon` / `lat` dimensions, otherwise the
    C++ Grid will index them against the wrong axis.
    """
    for axis_coord, dim in (("x_m", "lon"), ("y_m", "lat")):
        if axis_coord not in ds.coords:
            raise ValueError(
                f"{func_name}: dataset is missing coordinate '{axis_coord}'. "
                f"Caller must assign projected metric coordinates before exporting."
            )
        coord = ds.coords[axis_coord]
        if dim not in coord.dims:
            raise ValueError(
                f"{func_name}: coordinate '{axis_coord}' is not aligned with "
                f"dim '{dim}' (its dims are {coord.dims}). The metric grid will "
                f"be flattened against the wrong axis."
            )
        if len(coord) != len(ds.coords[dim]):
            raise ValueError(
                f"{func_name}: '{axis_coord}' length {len(coord)} does not match "
                f"'{dim}' length {len(ds.coords[dim])}."
            )


def _ensure_output_dir(output_dir: str | None) -> None:
    """Create `output_dir` (and any parents) if it does not exist."""
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)


def export_bathymetry(dataset: xr.Dataset,
                      x_filename: str = "x_coords.npy",
                      y_filename: str = "y_coords.npy",
                      bathy_filename: str = "bathymetry.npy",
                      output_dir: str | None = None) -> None:
    """Write Grid2D-compatible bathymetry npy files.

    Outputs (in `output_dir` if given, else cwd):
        x_coords.npy: 1D float64 array of x_m coordinates
        y_coords.npy: 1D float64 array of y_m coordinates
        bathymetry.npy: 1D float64 array, flattened with `ix * ny + iy` ordering

    Raises ValueError if the dataset's bathymetry dims or x_m/y_m coordinates
    are not arranged as the C++ Grid2D loader expects.
    """
    bathy_da = dataset["bathymetry"]
    _validate_dims(bathy_da, _BATHY_DIMS, "export_bathymetry")
    _validate_xy_coords(dataset, "export_bathymetry")
    _ensure_output_dir(output_dir)

    if output_dir is not None:
        x_filename = output_dir + x_filename
        y_filename = output_dir + y_filename
        bathy_filename = output_dir + bathy_filename

    print(f"Bathymetry dims: {bathy_da.dims}, shape: {bathy_da.shape}")
    x_coords = dataset.coords["x_m"].values.astype(np.float64)
    y_coords = dataset.coords["y_m"].values.astype(np.float64)
    print(f"x_m size: {len(x_coords)}, y_m size: {len(y_coords)}")

    # Grid2D expects x-major ordering: ix * ny + iy. Transpose to (lon, lat)
    # so the C-flatten produces that layout.
    bathymetry_flat = (
        bathy_da.transpose("lon", "lat").values.flatten(order="C").astype(np.float64)
    )

    np.save(x_filename, x_coords, allow_pickle=False)
    np.save(y_filename, y_coords, allow_pickle=False)
    np.save(bathy_filename, bathymetry_flat, allow_pickle=False)
    print(f"Exported x coords to {os.path.abspath(x_filename)}")
    print(f"Exported y coords to {os.path.abspath(y_filename)}")
    print(f"Exported bathymetry to {os.path.abspath(bathy_filename)}")


def export_ssp(dataset: xr.Dataset,
               x_filename: str = "x_coords.npy",
               y_filename: str = "y_coords.npy",
               z_filename: str = "depth_coords.npy",
               sound_speed_filename: str = "ssp.npy",
               output_dir: str | None = None) -> None:
    """Write Grid3D-compatible sound-speed npy files.

    Outputs (in `output_dir` if given, else cwd):
        x_coords.npy: 1D float64 array of x_m coordinates
        y_coords.npy: 1D float64 array of y_m coordinates
        depth_coords.npy: 1D float64 array of depth values
        ssp.npy: 1D float64 array, flattened with `ix * ny*nz + iy * nz + iz`

    Raises ValueError if the dataset's sound_speed dims or x_m/y_m coordinates
    are not arranged as the C++ Grid3D loader expects.
    """
    ssp_da = dataset["sound_speed"]
    _validate_dims(ssp_da, _FIELD_3D_DIMS, "export_ssp")
    _validate_xy_coords(dataset, "export_ssp")
    _ensure_output_dir(output_dir)

    if output_dir is not None:
        x_filename = output_dir + x_filename
        y_filename = output_dir + y_filename
        z_filename = output_dir + z_filename
        sound_speed_filename = output_dir + sound_speed_filename

    print(f"SSP dims: {ssp_da.dims}, shape: {ssp_da.shape}")
    x_coords = dataset.coords["x_m"].values.astype(np.float64)
    y_coords = dataset.coords["y_m"].values.astype(np.float64)
    depths = dataset.coords["depth"].values.astype(np.float64)
    print(f"x_m size: {len(x_coords)}, y_m size: {len(y_coords)}, depth size: {len(depths)}")

    sound_speed_flat = (
        ssp_da.transpose("lon", "lat", "depth")
        .values.flatten(order="C")
        .astype(np.float64)
    )

    np.save(x_filename, x_coords, allow_pickle=False)
    np.save(y_filename, y_coords, allow_pickle=False)
    np.save(z_filename, depths, allow_pickle=False)
    np.save(sound_speed_filename, sound_speed_flat, allow_pickle=False)
    print(f"Exported x coords to {os.path.abspath(x_filename)}")
    print(f"Exported y coords to {os.path.abspath(y_filename)}")
    print(f"Exported depth coords to {os.path.abspath(z_filename)}")
    print(f"Exported ssp to {os.path.abspath(sound_speed_filename)}")


def export_uv(dataset: xr.Dataset,
              x_filename: str = "x_coords.npy",
              y_filename: str = "y_coords.npy",
              u_filename: str = "u.npy",
              v_filename: str = "v.npy",
              z_filename: str = "z_coords.npy",
              output_dir: str | None = None) -> None:
    """Write GridVec-compatible current (u, v) npy files.

    Outputs (in `output_dir` if given, else cwd):
        x_coords.npy: 1D float64 array of x_m coordinates
        y_coords.npy: 1D float64 array of y_m coordinates
        z_coords.npy: 1D float64 array of depth values
        u.npy / v.npy: 1D float64 arrays flattened `ix * ny*nz + iy * nz + iz`

    NaN values in u/v are replaced with zero so the C++ interpolator never
    propagates missing data into the trajectory solver.

    Raises ValueError if u or v dims, or the x_m/y_m coordinates, are not
    arranged as the C++ GridVec loader expects.
    """
    u_da = dataset["u"]
    v_da = dataset["v"]
    _validate_dims(u_da, _FIELD_3D_DIMS, "export_uv (u)")
    _validate_dims(v_da, _FIELD_3D_DIMS, "export_uv (v)")
    _validate_xy_coords(dataset, "export_uv")
    _ensure_output_dir(output_dir)

    if output_dir is not None:
        x_filename = output_dir + x_filename
        y_filename = output_dir + y_filename
        u_filename = output_dir + u_filename
        v_filename = output_dir + v_filename
        z_filename = output_dir + z_filename

    print(f"Current u dims: {u_da.dims}, shape: {u_da.shape}")
    x_coords = dataset.coords["x_m"].values.astype(np.float64)
    y_coords = dataset.coords["y_m"].values.astype(np.float64)
    depths = dataset.coords["depth"].values.astype(np.float64)

    u = (
        u_da.transpose("lon", "lat", "depth")
        .values.flatten(order="C")
        .astype(np.float64)
    )
    v = (
        v_da.transpose("lon", "lat", "depth")
        .values.flatten(order="C")
        .astype(np.float64)
    )

    # Replace missing current data with zeros to prevent the C++ interpolator
    # from propagating NaN through the trajectory solver.
    u = np.nan_to_num(u, nan=0.0)
    v = np.nan_to_num(v, nan=0.0)

    np.save(x_filename, x_coords, allow_pickle=False)
    np.save(y_filename, y_coords, allow_pickle=False)
    np.save(u_filename, u, allow_pickle=False)
    np.save(v_filename, v, allow_pickle=False)
    np.save(z_filename, depths, allow_pickle=False)

    print(f"Exported x coords to {os.path.abspath(x_filename)}")
    print(f"Exported y coords to {os.path.abspath(y_filename)}")
    print(f"Exported z coords to {os.path.abspath(z_filename)}")
    print(f"Exported u to {os.path.abspath(u_filename)}")
    print(f"Exported v to {os.path.abspath(v_filename)}")
