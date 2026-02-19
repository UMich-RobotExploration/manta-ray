from oceanbench_data_provider import DataProvider, list_products, describe
from oceanbench_data_provider.viz import quicklook_map, make_movie, plot_region_bounds, interactive_globe
from oceanbench_data_provider.scenarios import list_scenarios, get_scenario, list_regions, get_region_bounds, get_region
from oceanbench_data_provider.cache.store import CacheStore
from oceanbench_data_provider import add_pressure_and_sound_speed
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import xarray as xr
import os


def export_bathymetry(dataset, x_filename="x_coords.npy", y_filename="y_coords.npy", bathy_filename="bathymetry.npy",
                      output_dir=None):
    """
    Args:
        dataset: weather model data
        x_filename: output for xcoords
        y_filename: see above
        bathy_filename: filename for bathymetry data
        output_dir: root of directory outputs should be saved in (None means in repo)
    Outputs:
        Export x, y coordinates and bathymetry data from the dataset to separate .npy files.
        - x_coords.npy: 1D array of x_m coordinates
        - y_coords.npy: 1D array of y_m coordinates
        - bathymetry.npy: 1D array of bathymetry values, flattened in C order
    """
    if output_dir is not None:
        x_filename = output_dir + x_filename
        y_filename = output_dir + y_filename
        bathy_filename = output_dir + bathy_filename
    x_coords = dataset.coords['x_m'].values.astype(np.float64)
    y_coords = dataset.coords['y_m'].values.astype(np.float64)
    bathymetry_flat = dataset['bathymetry'].values.flatten(order='C').astype(np.float64)
    np.save(x_filename, x_coords, allow_pickle=False)
    np.save(y_filename, y_coords, allow_pickle=False)
    np.save(bathy_filename, bathymetry_flat, allow_pickle=False)
    print(f"Exported x coords to {os.path.abspath(x_filename)}")
    print(f"Exported y coords to {os.path.abspath(y_filename)}")
    print(f"Exported bathymetry to {os.path.abspath(bathy_filename)}")


def export_ssp(dataset, x_filename="x_coords.npy", y_filename="y_coords.npy", z_filename="depth_coords.npy",
               sound_speed_filename="ssp.npy", output_dir=None):
    """
    Export x, y coordinates and bathymetry data from the dataset to separate .npy files.
    Args:
        dataset: weather model data
        x_filename: output for xcoords
        y_filename: see above
        z_filename: see above
        sound_speed_filename: filename for ssp data
        output_dir: root of directory outputs should be saved in (None means in repo)


    Outputs:
        x_coords.npy: 1D array of x_m coordinates
        y_coords.npy: 1D array of y_m coordinates
        z_coords.npy: 1D array of depth coordinates
        ssp_npy: 1D flattened array
    """
    if output_dir is not None:
        x_filename = output_dir + x_filename
        y_filename = output_dir + y_filename
        z_filename = output_dir + z_filename
        sound_speed_filename = output_dir + sound_speed_filename
    x_coords = dataset.coords['x_m'].values.astype(np.float64)
    y_coords = dataset.coords['y_m'].values.astype(np.float64)
    depths = dataset.coords['depth'].values.astype(np.float64)
    sound_speed_flat = dataset['sound_speed'].values.flatten(order='C').astype(np.float64)
    np.save(x_filename, x_coords, allow_pickle=False)
    np.save(y_filename, y_coords, allow_pickle=False)
    np.save(z_filename, depths, allow_pickle=False)
    np.save(sound_speed_filename, sound_speed_flat, allow_pickle=False)
    print(f"Exported x coords to {os.path.abspath(x_filename)}")
    print(f"Exported y coords to {os.path.abspath(y_filename)}")
    print(f"Exported depth coords to {os.path.abspath(z_filename)}")
    print(f"Exported ssp to {os.path.abspath(sound_speed_filename)}")


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


def plot_ssp(ds: xr.Dataset):
    fig, ax = plt.subplots(figsize=(4, 3), dpi=400)
    ax.yaxis.set_inverted(True)
    for i in ds['lat']:
        for j in ds['lon']:
            tmp_ds = ds.sel(lat=i, lon=j)
            ax.plot(tmp_ds['sound_speed'], tmp_ds['depth'])
    ax.set_title("Sound Speed Profile")
    ax.set_xlabel("Sound Speed (m/s)")
    ax.set_ylabel("Depth (m)")
    ax.minorticks_on()
    ax.grid(True)
    plt.show()


def main():
    # Initialize the data provider
    provider = DataProvider()

    SCENARIO_ID = "monterey_bay_phy"

    s = get_scenario(SCENARIO_ID)
    SELECTED_PRODUCT_ID = s.product_id
    REGION = s.region
    TIME_RANGE = s.time
    SELECTED_VARIABLES = s.variables
    # Load dataset card and derived info so later steps (estimate, fetch, plot) work
    card = describe(SELECTED_PRODUCT_ID)
    time_start = card.time_coverage_start
    time_end = card.time_coverage_end
    available_variables = [v.canonical_name for v in card.variables]
    variable_info = {v.canonical_name: v for v in card.variables}
    print(f"Scenario loaded: {s.name}")
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

    # Estimate size
    size_estimate = provider.estimate_size(
        SELECTED_PRODUCT_ID,
        REGION,
        TIME_RANGE,
        SELECTED_VARIABLES
    )

    print("Size Estimate:")
    print("=" * 60)
    if 'estimate_mb' in size_estimate:
        size_mb = size_estimate['estimate_mb']
        size_gb = size_mb / 1024
        print(f"Estimated size: {size_mb:.1f} MB ({size_gb:.2f} GB)")
    else:
        print(f"Estimate: {size_estimate}")

    if 'note' in size_estimate:
        print(f"\nNote: {size_estimate['note']}")

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

    from pyproj import Proj, Transformer

    # NOTE: Can use ds.isel(index_name=index_number) to select a special subset

    # Slicing out a uniform rectangle in the dataset before computing the center lat/lon
    new_ds = ds.isel(lat=slice(1, 7), lon=slice(0, 7))

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

    # Plot the map using the library `quicklook_map` helper
    for i in range(ds.depth.shape[0]):
        ax = quicklook_map(
            new_ds,
            "sound_speed",
            time_idx=0,
            depth_idx=i,
            variable_info=variable_info,
            show_date=True,
        )
        plt.show()
        break

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

    ax = quicklook_map(
        new_ds,
        "bathymetry",
        time_idx=0,
        depth_idx=i,
        variable_info=variable_info,
        show_date=True,
    )
    plt.show()
    plt.close()

    print(f"X Grid: {new_ds.coords['x_m'].values}")
    print(f"Y Grid: {new_ds.coords['y_m'].values}")
    print(f"Bathymetry Values: {new_ds['bathymetry'].values.flatten('C')}")

    output_dir = "../mantaray/data/monterey/bathymetry/"
    export_bathymetry(new_ds, output_dir=output_dir)
    output_dir = "../mantaray/data/monterey/ssp/"
    export_ssp(new_ds, output_dir=output_dir)


if __name__ == "__main__":
    main()
