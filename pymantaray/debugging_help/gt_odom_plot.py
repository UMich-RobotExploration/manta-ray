import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

root_dir ="/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src"
gt_file =  f"{root_dir}/simTest_0_GroundTruthPose.csv"
odom_file = f"{root_dir}/simTest_0_PosOdomXY.csv"
gt_df = pd.read_csv(gt_file)
odom_df = pd.read_csv(odom_file)
print(gt_df)
print(odom_df)

# Align DataFrames by index (assuming same length and order)
diff = np.sqrt((gt_df['x'] - odom_df['x'])**2 + (gt_df['y'] - odom_df['y'])**2)

time = gt_df['timestep'] if 'timestep' in gt_df.columns else gt_df.index

# --- Original plot (trajectories only) ---
plt.figure(figsize=(8, 8), dpi=150)
plt.plot(gt_df["x"], gt_df["y"], label="Ground Truth", marker='o')
plt.plot(odom_df["x"], odom_df["y"], label="Odom", marker='x')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.title('Trajectories')
plt.show()

# --- New plot: trajectories and difference ---
fig, axs = plt.subplots(2, 1, figsize=(10, 10), dpi=300, layout='tight')

# Plot trajectories
axs[0].plot(gt_df["x"], gt_df["y"], label="Ground Truth", marker='o')
axs[0].plot(odom_df["x"], odom_df["y"], label="Odom", marker='x')
axs[0].set_xlabel('x (m)')
axs[0].set_ylabel('y (m)')
axs[0].legend()
axs[0].set_title('Trajectories')

# Plot difference vs time
axs[1].plot(time, diff, label="Odom-GT Difference")
axs[1].set_xlabel('Time' if 'timestep' in gt_df.columns else 'Index')
axs[1].set_ylabel('Euclidean Distance (m)')
axs[1].legend()
axs[1].set_title('Odom vs GT Difference Over Time')

plt.show()
