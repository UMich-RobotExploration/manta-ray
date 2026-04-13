from oceanbench_data_provider import DataProvider, list_products, describe
from oceanbench_data_provider.viz import quicklook_map, make_movie, plot_region_bounds, interactive_globe
from oceanbench_data_provider.scenarios import list_scenarios, get_scenario, list_regions, get_region_bounds, get_region
from oceanbench_data_provider.cache.store import CacheStore
from oceanbench_data_provider import add_pressure_and_sound_speed
from pykrige.ok import OrdinaryKriging
from pykrige.ok3d import OrdinaryKriging3D
from pyproj import Proj, Transformer
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import xarray as xr

from grid_exporters import export_bathymetry, export_ssp, export_uv


################################################################################
# This is a scratch workbook and not true python code yet.

# TODO: Fix AI slop that generated most of this
################################################################################


# Local Arctic region presets — swap ACTIVE_REGION / ACTIVE_SEASON to change pulls.
# All entries use HYCOM glbv0.08 reanalysis 53.x (no credentials, ~80°N coverage limit).
ARCTIC_REGIONS = {
    "beaufort_sea": {
        "name": "Beaufort Sea",
        "region": {"lon": [-150, -130], "lat": [71, 76]},
        "roi_lon": (-150.0, -130.0),
        # "roi_lon": (-141.5, -140.5),
        # "roi_lat": (75.25, 76.0),
        "roi_lat": (73.0, 76.0),
        "variables": ["temp", "sal", "u", "v"],
        "product_id": "hycom_glbv0.08_reanalysis_53x",
        "output_subdir": "beaufort",
    },
    "fram_strait": {
        "name": "Fram Strait",
        "region": {"lon": [-10, 10], "lat": [76, 80]},
        "roi_lon": (-1.0, 0.0),
        "roi_lat": (78.0, 78.5),
        "variables": ["temp", "sal", "u", "v"],
        "product_id": "hycom_glbv0.08_reanalysis_53x",
        "output_subdir": "fram_strait",
    },
    "chukchi_sea": {
        "name": "Chukchi Sea",
        "region": {"lon": [-175, -155], "lat": [68, 73]},
        "roi_lon": (-167.0, -166.0),
        "roi_lat": (70.5, 71.0),
        "variables": ["temp", "sal", "u", "v"],
        "product_id": "hycom_glbv0.08_reanalysis_53x",
        "output_subdir": "chukchi",
    },
    "barents_sea": {
        "name": "Barents Sea",
        "region": {"lon": [20, 50], "lat": [70, 78]},
        "roi_lon": (34.0, 35.0),
        "roi_lat": (74.0, 74.5),
        "variables": ["temp", "sal", "u", "v"],
        "product_id": "hycom_glbv0.08_reanalysis_53x",
        "output_subdir": "barents",
    },
}

# HYCOM glbv0.08 53.x is 3-hourly; one timestep per season keeps downloads tiny.
SEASON_TIMES = {
    "winter": ("2014-02-15T00:00", "2014-02-15T03:00"),
    "spring": ("2014-04-15T00:00", "2014-04-15T03:00"),
    "summer": ("2014-07-15T00:00", "2014-07-15T03:00"),
    "autumn": ("2014-10-15T00:00", "2014-10-15T03:00"),
}

ACTIVE_REGION = "beaufort_sea"  # swap to fram_strait / chukchi_sea / barents_sea
ACTIVE_SEASON = "summer"        # swap to spring / summer / autumn

# Hard cap on local HYCOM fetches. Estimator runs before download; if the
# estimate exceeds this, the script aborts so we never accidentally pull GBs.
MAX_FETCH_MB = 500.0


def extract_roi(ds: xr.Dataset, lon_bounds: tuple[float, float],
                lat_bounds: tuple[float, float]) -> xr.Dataset:
    """Slice a dataset to a smaller lon/lat region of interest.

    Callers specify geographic bounds rather than index ranges so the same
    call works regardless of grid resolution or which region was pulled.
    """
    lon_lo, lon_hi = sorted(lon_bounds)
    lat_lo, lat_hi = sorted(lat_bounds)
    return ds.sel(lon=slice(lon_lo, lon_hi), lat=slice(lat_lo, lat_hi))


def estimate_hycom_size(region: dict, time: tuple[str, str],
                        variables: list[str], n_depth: int = 41) -> dict:
    """Accurate HYCOM size estimate that respects sub-day time windows.

    Upstream HycomAdapter.estimate_size() (deps/.../sources/hycom.py) computes
    n_time as (years_spanned * 365 * 8), so any request inside one year reports
    a full-year download (~25 GB) regardless of how short the actual window is.
    This helper computes n_time from the actual pd.Timestamp delta at HYCOM's
    3-hour cadence, so a 3-hour pull reports ~10 MB.
    """
    lon_b = region["lon"]
    lat_b = region["lat"]
    n_lat = max(1, int((lat_b[1] - lat_b[0]) * 12))   # 1/12 deg
    n_lon = max(1, int((lon_b[1] - lon_b[0]) * 12))
    start, end = pd.Timestamp(time[0]), pd.Timestamp(time[1])
    hours = max(0.0, (end - start).total_seconds() / 3600.0)
    # HYCOM 53.x is 3-hourly; +1 covers the endpoint inclusivity of xarray's slice.
    n_time = max(1, int(hours // 3) + 1)
    n_vars = len(variables)
    bytes_per_value = 4
    total = n_lat * n_lon * n_time * n_depth * n_vars * bytes_per_value
    return {
        "estimate_bytes": total,
        "estimate_mb": total / (1024 * 1024),
        "n_lat": n_lat,
        "n_lon": n_lon,
        "n_time": n_time,
        "n_depth": n_depth,
        "n_vars": n_vars,
        "note": "Local estimate (upstream HycomAdapter.estimate_size ignores sub-year windows).",
    }


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
            nan_idx = np.argmax(np.isnan(sound_speed_profile))  # Find the first NaN index
            if nan_idx > 0 or (np.isnan(sound_speed_profile[0]).any()):  # Ensure NaN exists
                bathymetry[i, j] = ds.coords['depth'][nan_idx].values - epsilon
            else:  # If no NaN, take the maximum depth
                bathymetry[i, j] = ds.coords['depth'].values[-1] - epsilon
    return bathymetry


def compute_ssp(ds, var='sound_speed'):
    """
    Fill NaNs below the last valid value along the 'depth' axis for each (lat, lon) using xarray ops.
    The last valid value is extended downward to fill all NaNs below it.
    """
    arr = ds[var]
    # Forward-fill along depth (from shallow to deep)
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


def krige_data_2d(ds, var_name='bathymetry', fine_factor=5):
    """
    Perform Ordinary Kriging on the bathymetry field and return a new DataArray with kriged values on a finer x_m/y_m grid.
    Also creates lat/lon coordinates for the fine grid using np.linspace.
    """
    x = ds.coords['x_m'].values
    y = ds.coords['y_m'].values
    bathy = ds[var_name].values
    # Kriging on fine grid
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

    # Return as DataArray with new finer coordinates
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
    # Transpose data to (lat, lon, depth) before flattening
    data = ds[var_name].values.transpose(1, 2, 0)
    print(f"x shape: {x.shape}, y shape: {y.shape}, z shape: {z.shape}, data shape (lat, lon, depth): {data.shape}")
    # Flatten the grid for kriging input
    xx, yy, zz = np.meshgrid(x, y, z, indexing='ij')
    print(f"meshgrid shapes: xx {xx.shape}, yy {yy.shape}, zz {zz.shape}")
    x_flat = xx.flatten()
    y_flat = yy.flatten()
    z_flat = zz.flatten()
    data_flat = data.flatten()
    # Remove NaNs for kriging
    mask = ~np.isnan(data_flat)
    x_valid = x_flat[mask]
    y_valid = y_flat[mask]
    z_valid = z_flat[mask]
    data_valid = data_flat[mask]
    print(f"Valid points: {x_valid.shape}")
    # Set up the kriging object
    OK = OrdinaryKriging3D(
        x_valid, y_valid, z_valid, data_valid,
        variogram_model='spherical',
        verbose=False, enable_plotting=True
    )
    # Interpolate on the original grid
    z_kriged, _ = OK.execute('grid', x, y, z)
    print(f"z_kriged shape (should be z,y,x): {z_kriged.shape}")
    # Transpose to (lat, lon, depth) = (y, x, z)
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


def main():
    # Initialize the data provider
    provider = DataProvider()

    cfg = ARCTIC_REGIONS[ACTIVE_REGION]
    SELECTED_PRODUCT_ID = cfg["product_id"]
    REGION = cfg["region"]
    TIME_RANGE = SEASON_TIMES[ACTIVE_SEASON]
    SELECTED_VARIABLES = cfg["variables"]
    # Load dataset card and derived info so later steps (estimate, fetch, plot) work
    card = describe(SELECTED_PRODUCT_ID)
    time_start = card.time_coverage_start
    time_end = card.time_coverage_end
    available_variables = [v.canonical_name for v in card.variables]
    variable_info = {v.canonical_name: v for v in card.variables}
    print(f"Scenario loaded: {cfg['name']} ({ACTIVE_SEASON})")
    print(f"  Product: {SELECTED_PRODUCT_ID}")
    print(f"  Region: lon={REGION['lon']}, lat={REGION['lat']}")
    print(f"  Time: {TIME_RANGE[0]} to {TIME_RANGE[1]}")
    print(f"  Variables: {', '.join(SELECTED_VARIABLES)}")
    print("\nYou can skip to Step 7 (Estimate data size).")

    # Extract time coverage
    time_start = card.time_coverage_start
    time_end = card.time_coverage_end

    # Extract available variables
    available_variables = [v.canonical_name for v in card.variables]
    variable_info = {v.canonical_name: v for v in card.variables}

    print(f"\nTime Coverage: {time_start} to {time_end}")
    print(f"  Temporal Resolution: {card.temporal_resolution}")
    print(f"  Spatial Resolution: {card.spatial_resolution}")
    print(f"\nAvailable Variables ({len(available_variables)}):")
    for var in available_variables:
        vinfo = variable_info[var]
        dim_type = "2D (surface)" if vinfo.is_2d else "3D (depth-resolved)"
        print(f"  • {var:10s} - {vinfo.description:40s} [{vinfo.units:10s}] ({dim_type})")

    # Estimate size locally (upstream HycomAdapter.estimate_size is buggy and
    # always reports a full-year download for any sub-year window).
    size_estimate = estimate_hycom_size(REGION, TIME_RANGE, SELECTED_VARIABLES)
    size_mb = size_estimate["estimate_mb"]
    size_gb = size_mb / 1024

    print("Size Estimate (local):")
    print("=" * 60)
    print(f"Estimated size: {size_mb:.1f} MB ({size_gb:.3f} GB)")
    print(f"  grid: {size_estimate['n_lat']} lat × {size_estimate['n_lon']} lon "
          f"× {size_estimate['n_depth']} depth × {size_estimate['n_time']} time × "
          f"{size_estimate['n_vars']} vars")
    print(f"\nNote: {size_estimate['note']}")

    if size_mb > MAX_FETCH_MB:
        raise RuntimeError(
            f"Estimated fetch size {size_mb:.1f} MB exceeds MAX_FETCH_MB="
            f"{MAX_FETCH_MB:.0f} MB. Narrow REGION or TIME_RANGE, or raise the cap."
        )

    print(f"\nVariables: {', '.join(SELECTED_VARIABLES)}")
    print(f"Region: lon={REGION['lon']}, lat={REGION['lat']}")
    print(f"Time: {TIME_RANGE[0]} to {TIME_RANGE[1]}")

    print("Fetching data...")
    print(f"   Product: {SELECTED_PRODUCT_ID}")
    print(f"   Variables: {', '.join(SELECTED_VARIABLES)}")
    print(f"   Region: {REGION}")
    print(f"   Time: {TIME_RANGE[0]} to {TIME_RANGE[1]}")
    print("\nThis may take a while for large datasets...")

    # Fetch data
    ds = provider.subset(
        product_id=SELECTED_PRODUCT_ID,
        region=REGION,
        time=TIME_RANGE,
        variables=SELECTED_VARIABLES,
        overwrite=False  # Set to True to re-fetch even if cached
    )

    print("\nData fetched successfully!")

    print(f"Cache location: {CacheStore().root}")

    # Show the full pulled region on a polar-stereo Arctic map for visual
    # confirmation of *where* the fetch landed before any ROI extraction.
    fig, ax = make_arctic_axes()
    quicklook_map(
        ds,
        "temp",
        ax=ax,
        time_idx=0,
        depth_idx=0,
        variable_info=variable_info,
        show_date=True,
    )
    ax.set_title(f"{cfg['name']} surface temperature — full pull on Arctic basin")
    plt.show()
    plt.close(fig)

    # Requirements:
    # - ds has variables "temp" and "sal"
    # - ds has coordinate "depth" (in meters, positive downward)
    # - ds has coordinate "lat" (in degrees north)
    # If these are satisfied, you can safely run this cell.

    required = ["temp", "sal"]
    missing = [v for v in required if v not in ds.data_vars]
    if "depth" not in ds.coords or "lat" not in ds.coords or missing:
        print("Cannot add pressure/sound speed automatically.")
        print("   Needed: variables temp & sal, coords depth & lat.")
        print("   Missing:", missing + [c for c in ["depth", "lat"] if c not in ds.coords])
    else:
        ds = add_pressure_and_sound_speed(ds)
        print("Added variables: pressure, sound_speed (TEOS-10, via gsw).")
        print("\nAvailable data variables after adding TEOS-10 fields:")
        for name in ds.data_vars:
            print(f"  - {name}")

    print("Dataset Information:")
    print("=" * 60)
    # print(f"Dimensions: {dict(ds.dims)}")
    print(f"\nData Variables: {list(ds.data_vars)}")
    print(f"\nCoordinates:")
    for coord in ds.coords:
        coord_data = ds.coords[coord]
        if 'time' in coord.lower():
            print(f"  • {coord}: {len(coord_data)} time steps from {coord_data.values[0]} to {coord_data.values[-1]}")
        elif 'lon' in coord.lower():
            print(
                f"  • {coord}: {len(coord_data)} points from {coord_data.values.min():.2f}° to {coord_data.values.max():.2f}°")
        elif 'lat' in coord.lower():
            print(
                f"  • {coord}: {len(coord_data)} points from {coord_data.values.min():.2f}° to {coord_data.values.max():.2f}°")
        elif 'depth' in coord.lower():
            print(
                f"  • {coord}: {len(coord_data)} levels from {coord_data.values.min():.1f}m to {coord_data.values.max():.1f}m")
        else:
            print(f"  • {coord}: {len(coord_data)} values")

    # Carve a smaller geographic ROI out of the fetched dataset for downstream experiments.
    new_ds = extract_roi(ds, cfg["roi_lon"], cfg["roi_lat"])

    # Define the center of the sliced region
    center_lon, center_lat = (new_ds.coords['lon'].values[0] + new_ds.coords['lon'].values[-1]) / 2, \
                             (new_ds.coords['lat'].values[0] + new_ds.coords['lat'].values[-1]) / 2

    # Define a local projection centered on the sliced region
    local_proj = Proj(proj="aeqd", lat_0=center_lat, lon_0=center_lon, ellps="WGS84")

    # Create a transformer for converting lat/lon to the local coordinate system
    transformer = Transformer.from_crs("EPSG:4326", local_proj.crs, always_xy=True)

    # Convert the dataset's coordinates to meters
    lon, lat = new_ds.coords['lon'].values, new_ds.coords['lat'].values
    lon_grid, lat_grid = np.meshgrid(lon, lat)
    x, y = transformer.transform(lon_grid, lat_grid)

    # Replace the coordinates in the sliced dataset
    new_ds = new_ds.assign_coords(x_m=("lon", x[0, :]), y_m=("lat", y[:, 0]))
    new_ds = new_ds.isel(time=0)

    print("Grid converted to meters.")

    # Define a small epsilon offset
    epsilon = 2.5  # meter offset

    # Compute bathymetry
    bathymetry = compute_bathymetry(new_ds, epsilon=epsilon)

    # Add the bathymetry field to the dataset
    new_ds['bathymetry'] = (('lat', 'lon'), bathymetry)

    plot_ssp(new_ds)
    filled_ssp = compute_ssp(new_ds)
    new_ds['sound_speed'] = filled_ssp
    plot_ssp(new_ds)

    print("Bathymetry field added to the dataset.")

    fig, ax = make_arctic_axes()
    quicklook_map(
        new_ds,
        "bathymetry",
        ax=ax,
        time_idx=0,
        depth_idx=0,
        variable_info=variable_info,
        show_date=True,
    )
    ax.set_title(f"{cfg['name']} bathymetry ({ACTIVE_SEASON}) — ROI on Arctic basin")
    plt.show()
    plt.close(fig)

    print(f"X Grid: {new_ds.coords['x_m'].values}")
    print(f"Y Grid: {new_ds.coords['y_m'].values}")
    print(f"Bathymetry Values: {new_ds['bathymetry'].values.flatten('C')}")

    out_root = f"../../mantaray/data/{cfg['output_subdir']}_{ACTIVE_SEASON}"
    output_dir = f"{out_root}/bathymetry/"
    export_bathymetry(new_ds, output_dir=output_dir)
    output_dir = f"{out_root}/ssp/"
    export_ssp(new_ds, output_dir=output_dir)
    output_dir = f"{out_root}/current/"
    export_uv(new_ds, output_dir=output_dir)

    # # After exporting bathymetry and ssp, perform kriging and plot
    # bathy_kriged = krige_data_2d(new_ds, var_name='bathymetry')
    # # Create a new dataset for kriged bathymetry
    # kriged_ds = new_ds.copy()
    # kriged_ds['bathymetry_kriged'] = bathy_kriged
    # print("Plotting kriged bathymetry...")
    # ax = quicklook_map(
    #     kriged_ds,
    #     "bathymetry_kriged",
    #     time_idx=0,
    #     depth_idx=0,
    #     variable_info=variable_info,
    #     show_date=True,
    # )
    # plt.show()
    #
    # ssp_krige = krige_data_3d(new_ds, var_name='sound_speed')
    # kriged_ds['sound_speed_kriged'] = ssp_krige
    # plot_ssp(kriged_ds, 'sound_speed_kriged')


if __name__ == "__main__":
    main()
