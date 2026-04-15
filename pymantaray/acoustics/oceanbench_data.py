from oceanbench_data_provider import DataProvider, list_products, describe
from oceanbench_data_provider.viz import quicklook_map, make_movie, plot_region_bounds, interactive_globe
from oceanbench_data_provider.scenarios import list_scenarios, get_scenario, list_regions, get_region_bounds, get_region
from oceanbench_data_provider.cache.store import CacheStore
from oceanbench_data_provider import add_pressure_and_sound_speed
from pyproj import Proj, Transformer
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from grid_exporters import export_bathymetry, export_ssp, export_uv
from oceanbench_process import (
    compute_bathymetry,
    compute_ssp,
    extract_roi,
    krige_data_2d,
    krige_data_3d,
    make_arctic_axes,
    plot_ssp,
    plot_ssp_bounds,
)


################################################################################
# This is a scratch workbook and not true python code yet.

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

    plot_ssp_bounds(new_ds)
    plt.show()
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
