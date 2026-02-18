import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from glob import glob

files = glob("arrivalDataDebug/*.csv")

agg_delta_error = np.zeros((len(files),))
agg_percent_error = np.zeros((len(files),))
# Offset value for ground truth
OFFSET = 1
for i,file in enumerate(files):
    df = pd.read_csv(file)
    pd.set_option('display.precision', 10)
    print(df)
    range_series_min = df["ArrivalTime(s)"][OFFSET:].argmin()
    index = range_series_min + OFFSET
    true_range = df["Range(m)"][0]
    measured_range = df["Range(m)"][index]
    delta_error = measured_range - true_range
    agg_percent_error[i] = delta_error / true_range * 100
    agg_delta_error[i] = delta_error

fig, ax=plt.subplots(dpi=400, layout="tight")
ax.hist(agg_delta_error)
ax.set_xlabel("Range Error (m)")
ax.set_ylabel("Count")
fig.savefig("arrivalDataDebug/overall_range_error_hist.png")

fig, ax=plt.subplots(dpi=400, layout="tight")
ax.hist(agg_percent_error)
ax.set_xlabel("Percent Error")
ax.set_ylabel("Count")
fig.savefig("arrivalDataDebug/overall_range_percent_error_hist.png")



for file in files:
    df = pd.read_csv(file)
    print(df)
    range_series = df["Range(m)"]
    amplitude_series = df["Amplitude"]
    plt.figure(figsize=(10, 6), dpi=300)
    plt.scatter(range_series[1:], amplitude_series[1:])
    plt.vlines(range_series[0],np.min(amplitude_series[1:]),np.max(amplitude_series[1:]), 'r', '-.')
    plt.xlabel('Range [m]')
    plt.ylabel('Amplitude')
    plt.title(f"{file}, # Arrival: {len(df) - 1}, True Range: {range_series[0]}")
    x_min, x_max = range_series.min(), range_series.max()
    if x_max - x_min < 1:
        mid_point = (x_max + x_min) / 2
        x_min, x_max = mid_point - 0.5, mid_point + 0.5
    else:
        x_min = x_min * 0.99
        x_max = x_max * 1.01

    plt.xlim(x_min, x_max)
    png_file = file.strip().replace('.csv', '.png')
    plt.savefig(png_file)
    plt.close()
