from oceanbench_data_provider import DataProvider, list_products, describe

from oceanbench_data_provider import add_pressure_and_sound_speed
from oceanbench_data_provider.viz import (
    quicklook_map,
    make_movie,
    plot_region_bounds,
    interactive_globe,
)
from oceanbench_data_provider.scenarios import (
    list_scenarios,
    get_scenario,
    list_regions,
    get_region_bounds,
    get_region,
)
from oceanbench_data_provider.cache.store import CacheStore
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import xarray as xr

# Initialize the data provider
provider = DataProvider()


# List all available products
products = list_products()

print("Available OceanBench Products:")
print("=" * 50)
for i, p in enumerate(products, 1):
    print(f"{i:2d}. {p}")

print(f"\nTotal: {len(products)} products available")


# List available preset scenarios
# scenarios = list_scenarios()
# print("Available preset scenarios:")
# print("=" * 50)
# for i, sid in enumerate(scenarios, 1):
#     s = get_scenario(sid)
#     print(f"  {i}. {sid}")
#     print(f"     {s.name}: {s.description or '(product, region, time, variables)'}")
# print()

SCENARIO_ID = "monterey_bay_phy"

s = get_scenario(SCENARIO_ID)
SELECTED_PRODUCT_ID = s.product_id
REGION = s.region
# s.time = (s.time[0], "2014-06-03")
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

print("🔄 Fetching data...")
print(f"   Product: {SELECTED_PRODUCT_ID}")
print(f"   Variables: {', '.join(SELECTED_VARIABLES)}")
print(f"   Region: {REGION}")
print(f"   Time: {TIME_RANGE[0]} to {TIME_RANGE[1]}")
print("\nThis may take a while for large datasets...")

# Estimate size
size_estimate = provider.estimate_size(
    SELECTED_PRODUCT_ID, REGION, TIME_RANGE, SELECTED_VARIABLES
)

print("📊 Size Estimate:")
print("=" * 60)
if "estimate_mb" in size_estimate:
    size_mb = size_estimate["estimate_mb"]
    size_gb = size_mb / 1024
    print(f"Estimated size: {size_mb:.1f} MB ({size_gb:.2f} GB)")
else:
    print(f"Estimate: {size_estimate}")

if "note" in size_estimate:
    print(f"\nNote: {size_estimate['note']}")

print(f"\nVariables: {', '.join(SELECTED_VARIABLES)}")
print(f"Region: lon={REGION['lon']}, lat={REGION['lat']}")
print(f"Time: {TIME_RANGE[0]} to {TIME_RANGE[1]}")

# Fetch data
ds = provider.subset(
    product_id=SELECTED_PRODUCT_ID,
    region=REGION,
    time=TIME_RANGE,
    variables=SELECTED_VARIABLES,
    overwrite=False,  # Set to True to re-fetch even if cached
)

print("\n✅ Data fetched successfully!")

print(f"Cache location: {CacheStore().root}")


ds = add_pressure_and_sound_speed(ds)



print("📋 Dataset Information:")
print("=" * 60)
print(f"Dimensions: {dict(ds.dims)}")
print(f"\nData Variables: {list(ds.data_vars)}")
print(f"\nCoordinates:")
for coord in ds.coords:
    coord_data = ds.coords[coord]
    if 'time' in coord.lower():
        print(f"  • {coord}: {len(coord_data)} time steps from {coord_data.values[0]} to {coord_data.values[-1]}")
    elif 'lon' in coord.lower():
        print(f"  • {coord}: {len(coord_data)} points from {coord_data.values.min():.2f}° to {coord_data.values.max():.2f}°")
    elif 'lat' in coord.lower():
        print(f"  • {coord}: {len(coord_data)} points from {coord_data.values.min():.2f}° to {coord_data.values.max():.2f}°")
    elif 'depth' in coord.lower():
        print(f"  • {coord}: {len(coord_data)} levels from {coord_data.values.min():.1f}m to {coord_data.values.max():.1f}m")
    else:
        print(f"  • {coord}: {len(coord_data)} values")



for i in range(len(ds.depth)):
    # Plot the map using the library `quicklook_map` helper
    ax = quicklook_map(
        ds,
        "sound_speed",
        time_idx=0,
        depth_idx=i,
        variable_info=variable_info,
        show_date=True,
    )
    plt.show()


