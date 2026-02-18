import numpy
import matplotlib.pyplot as plt
import pandas as pd

root_dir ="/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src"
gt_file =  f"{root_dir}/simTest_0_GroundTruthPose.csv"
odom_file = f"{root_dir}/simTest_0_PosOdomXY.csv"
gt_df = pd.read_csv(gt_file)
odom_df = pd.read_csv(odom_file)
print(gt_df)
print(odom_df)


fig = plt.figure(figsize=(10, 6), dpi=300, layout='tight')
plt.plot(gt_df["x"], gt_df["y"], label="Ground Truth", marker='o')
plt.plot(odom_df["x"], odom_df["y"], label="Odom", marker='x')
plt.xlabel('x (m)')
plt.ylabel('y (m)')
plt.legend()
plt.show()


