"""Transform and visualization helpers for standard OceanBench datasets.

Functions here expect an xarray Dataset with `lat`, `lon`, and `depth`
coords and a `sound_speed` variable (the shape produced by the fetch +
`add_pressure_and_sound_speed` pipeline in `oceanbench_data.py`).
"""

import matplotlib.pyplot as plt
import numpy as np
import xarray as xr
from pykrige.ok import OrdinaryKriging
from pykrige.ok3d import OrdinaryKriging3D


def extract_roi(ds: xr.Dataset, lon_bounds: tuple[float, float],
                lat_bounds: tuple[float, float]) -> xr.Dataset:
    """Slice a dataset to a smaller lon/lat region of interest.

    Callers specify geographic bounds rather than index ranges so the same
    call works regardless of grid resolution or which region was pulled.
    """
    lon_lo, lon_hi = sorted(lon_bounds)
    lat_lo, lat_hi = sorted(lat_bounds)
    return ds.sel(lon=slice(lon_lo, lon_hi), lat=slice(lat_lo, lat_hi))


def compute_bathymetry(ds, epsilon=2.5):
    """
    Compute bathymetry for a dataset by finding the first NaN in the sound speed profile along the depth dimension.
    If no NaN is found, use the maximum depth. Returns a 2D numpy array.

    Args:
        epsilon: The negative offset to apply to ensure bathymetry is slightly higher than last defined point

    Example. if epsilon = 1, and last depth holding value is 200 meters. The bathymetry is defined at 199 meters. This
    ensure SSP data is available within a small epsilon of the bathymetry.
    """
    bathymetry = np.full((len(ds.coords['lat']), len(ds.coords['lon'])), np.nan)
    for i, lat in enumerate(ds.coords['lat']):
        for j, lon in enumerate(ds.coords['lon']):
            sound_speed_profile = ds['sound_speed'].isel(lat=i, lon=j).values
            nan_idx = np.argmax(np.isnan(sound_speed_profile))
            if nan_idx > 0 or (np.isnan(sound_speed_profile[0]).any()):
                bathymetry[i, j] = ds.coords['depth'][nan_idx].values - epsilon
            else:
                bathymetry[i, j] = ds.coords['depth'].values[-1] - epsilon
    return bathymetry


def compute_ssp(ds, var='sound_speed'):
    """
    Fill NaNs below the last valid value along the 'depth' axis for each (lat, lon) using xarray ops.
    The last valid value is extended downward to fill all NaNs below it.
    """
    arr = ds[var]
    filled = arr.ffill(dim='depth')
    return filled


def plot_ssp(ds: xr.Dataset, xarray_key='sound_speed'):
    fig, ax = plt.subplots(figsize=(4, 3), dpi=400)
    ax.yaxis.set_inverted(True)
    for i in ds['lat']:
        for j in ds['lon']:
            tmp_ds = ds.sel(lat=i, lon=j)
            ax.plot(tmp_ds[xarray_key], tmp_ds['depth'])
    ax.set_title("Sound Speed Profile")
    ax.set_xlabel("Sound Speed (m/s)")
    ax.set_ylabel("Depth (m)")
    ax.minorticks_on()
    ax.grid(True)
    plt.show()


def plot_ssp_bounds(
    ds: xr.Dataset,
    var: str = "sound_speed",
    show_mean: bool = True,
    ax: plt.Axes | None = None,
) -> plt.Axes:
    """Plot the per-depth min/max envelope of an SSP ensemble.

    Collapses the (lat, lon) grid into a single ensemble and draws a
    shaded band spanning the minimum and maximum sound speed at each
    depth, optionally with the per-depth mean overlaid.
    """
    arr = ds[var]
    stacked = arr.stack(profile=("lat", "lon"))
    ssp_min = stacked.min(dim="profile", skipna=True)
    ssp_max = stacked.max(dim="profile", skipna=True)
    depth = ds["depth"]

    if ax is None:
        _, ax = plt.subplots(figsize=(4, 3), dpi=400)

    ax.fill_betweenx(depth, ssp_min, ssp_max, alpha=0.3, label="min/max")
    if show_mean:
        ssp_mean = stacked.mean(dim="profile", skipna=True)
        ax.plot(ssp_mean, depth, color="C0", linewidth=1.0, label="mean")
        ax.legend(loc="best", fontsize="small")

    ax.yaxis.set_inverted(True)
    ax.set_title("SSP min/max bounds")
    ax.set_xlabel("Sound Speed (m/s)")
    ax.set_ylabel("Depth (m)")
    ax.minorticks_on()
    ax.grid(True)
    return ax


def krige_data_2d(ds, var_name='bathymetry', fine_factor=5):
    """
    Perform Ordinary Kriging on the bathymetry field and return a new DataArray with kriged values on a finer x_m/y_m grid.
    Also creates lat/lon coordinates for the fine grid using np.linspace.
    """
    x = ds.coords['x_m'].values
    y = ds.coords['y_m'].values
    bathy = ds[var_name].values
    xx, yy = np.meshgrid(x, y)
    x_flat = xx.flatten()
    y_flat = yy.flatten()
    bathy_flat = bathy.flatten()
    mask = ~np.isnan(bathy_flat)
    x_valid = x_flat[mask]
    y_valid = y_flat[mask]
    bathy_valid = bathy_flat[mask]
    OK = OrdinaryKriging(
        x_valid, y_valid, bathy_valid,
        variogram_model='spherical',
        verbose=False, enable_plotting=True
    )
    x_fine = np.linspace(x.min(), x.max(), len(x) * fine_factor)
    y_fine = np.linspace(y.min(), y.max(), len(y) * fine_factor)
    xx, yy = np.meshgrid(x_fine, y_fine)
    z_kriged, _ = OK.execute('grid', xx.flatten(), yy.flatten())

    plt.figure(figsize=(8, 6))
    plt.contourf(xx.flatten(), yy.flatten(), z_kriged, 20, cmap='viridis')
    plt.colorbar(label='Depth')
    plt.title('Kriging Interpolation of Bathymetry (Higher Resolution)')
    plt.xlabel('X-coordinate')
    plt.ylabel('Y-coordinate')
    plt.show()

    print(z_kriged.shape)
    kriged_da = xr.DataArray(
        z_kriged,
        coords={
            'y_m': yy.flatten(),
            'x_m': xx.flatten(),
        },
        dims=('y_m', 'x_m')
    )
    return kriged_da


def krige_data_3d(ds, var_name='sound_speed'):
    """
    Perform Ordinary Kriging on the sound_speed field and return a new DataArray with kriged values.
    """
    x = ds.coords['x_m'].values
    y = ds.coords['y_m'].values
    z = ds.coords['depth'].values
    data = ds[var_name].values.transpose(1, 2, 0)
    print(f"x shape: {x.shape}, y shape: {y.shape}, z shape: {z.shape}, data shape (lat, lon, depth): {data.shape}")
    xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
    print(f"meshgrid shapes: xx {xx.shape}, yy {yy.shape}, zz {zz.shape}")
    x_flat = xx.flatten()
    y_flat = yy.flatten()
    z_flat = zz.flatten()
    data_flat = data.flatten()
    mask = ~np.isnan(data_flat)
    x_valid = x_flat[mask]
    y_valid = y_flat[mask]
    z_valid = z_flat[mask]
    data_valid = data_flat[mask]
    print(f"Valid points: {x_valid.shape}")
    OK = OrdinaryKriging3D(
        x_valid, y_valid, z_valid, data_valid,
        variogram_model='spherical',
        verbose=False, enable_plotting=True
    )
    z_kriged, _ = OK.execute('grid', x, y, z)
    print(f"z_kriged shape (should be z,y,x): {z_kriged.shape}")
    z_kriged = np.transpose(z_kriged, (1, 2, 0))
    print(f"z_kriged transposed shape (should be y,x,z): {z_kriged.shape}")
    print(f"lat: {ds.coords['lat'].shape}, lon: {ds.coords['lon'].shape}, depth: {ds.coords['depth'].shape}")
    kriged_da = xr.DataArray(z_kriged,
                             coords={'lat': ds.coords['lat'], 'lon': ds.coords['lon'], 'depth': ds.coords['depth']},
                             dims=('lat', 'lon', 'depth'))
    return kriged_da


def make_arctic_axes(figsize: tuple[float, float] = (10, 8)):
    """Create a NorthPolarStereo cartopy axes covering the Arctic basin.

    Pass the returned axes as `ax=` to `quicklook_map` so plotted data appears
    at its true geographic location while the surrounding Arctic ocean and
    coastlines remain visible. Extent is 60°N-90°N, full longitude — wide
    enough to contain all four `ARCTIC_REGIONS` presets.
    """
    import cartopy.crs as ccrs
    fig, ax = plt.subplots(
        figsize=figsize,
        subplot_kw=dict(projection=ccrs.NorthPolarStereo()),
    )
    ax.set_extent([-180, 180, 60, 90], crs=ccrs.PlateCarree())
    return fig, ax
