import open3d as o3d
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from aubellhop import readers
from aubellhop import plot
from aubellhop.environment import Environment 
from readers_1_8 import read_bty_3d

file_path = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/overhaul.ray"
arr_file_path = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/overhaul.arr"
env_file_path = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/overhaul.env"
bty_file_path = "/home/tko/repos/manta-ray/mantaray/cmake-build-release/src/overhaul.bty"
rays: pd.DataFrame = readers.read_rays(file_path)
# for i in range(rays.ray.iloc[0].shape[0]):
#     print(rays.ray.iloc[0][i])
# print(rays.ray[0].)
bty = read_bty_3d(bty_file_path)
dim_x, dim_y = bty['depths'].shape
print(bty['depths'].shape)
print(bty)
print(dim_x,dim_y)
x_vals = np.linspace(bty['ranges'][0], bty['ranges'][1], num=dim_x, endpoint=True)
y_vals = np.linspace(bty['crossranges'][0], bty['crossranges'][1], num=dim_y, endpoint=True)
Z = np.array(bty['depths'])  # keep depths positive
X, Y = np.meshgrid(x_vals, y_vals, indexing='ij')

# Create vertices from the grid
vertices = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

# Build triangles
triangles = []
for i in range(dim_x - 1):       # iterate over x
    for j in range(dim_y - 1):   # iterate over y
        # Vertex indices for current grid cell
        v0 = i * dim_y + j
        v1 = v0 + 1
        v2 = (i + 1) * dim_y + j
        v3 = v2 + 1

        # Two triangles per grid cell
        triangles.append([v0, v1, v2])
        triangles.append([v1, v3, v2])

triangles = np.array(triangles)

# Create Open3D mesh
mesh = o3d.geometry.TriangleMesh()
mesh.vertices = o3d.utility.Vector3dVector(vertices)
mesh.triangles = o3d.utility.Vector3iVector(triangles)
mesh.compute_vertex_normals()

# Color by depth (using viridis colormap)
z_norm = (Z.ravel() - Z.min()) / (Z.max() - Z.min())
colors = plt.cm.viridis(z_norm)[:, :3]  # use viridis colormap
mesh.vertex_colors = o3d.utility.Vector3dVector(colors)

# Add coordinate frame
frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10000)

# Create point cloud from rays
points = []
for curr_ray in rays.ray:
    points.extend(curr_ray)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

# Visualize with camera looking from +z direction
vis = o3d.visualization.Visualizer()
vis.create_window()
vis.add_geometry(mesh)
vis.add_geometry(pcd)
vis.add_geometry(frame)

# Set camera to look down from +z direction
ctr = vis.get_view_control()
ctr.set_front([0, 0, -1])  # look in -z direction (down at surface)
ctr.set_up([0, 1, 0])      # y-axis points up in view
ctr.set_lookat([np.mean(x_vals), np.mean(y_vals), np.mean(Z)])  # look at center

vis.run()
vis.destroy_window()

