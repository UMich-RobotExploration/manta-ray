"""Debug visualization for bellhop ray trace output.

Visualizes bathymetry mesh with source/receiver markers and optional ray traces.
Edit file_root below to point to your debug output (no .env extension).
"""

import os
import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from aubellhop import readers
from readers_1_8 import read_bty_3d

# ---- Edit this path (no .env extension) ----
# file_root = "/home/tko/repos/manta-ray/mantaray/cmake-build-debug/src/results/lbl/debug_R0_to_L2_7200s"
# file_root = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl/debug_R1_to_L2_19800s"
file_root = "/media/veracrypt1/College/Grad School/thesis/baseline-lbl/lbl-simple/noarrival_R0_to_L2_9000s"


def build_bathy_mesh(bty_path: str) -> o3d.geometry.TriangleMesh:
    bty = read_bty_3d(bty_path)
    x_vals = bty["ranges"]
    y_vals = bty["crossranges"]
    Z = np.array(bty["depths"])
    X, Y = np.meshgrid(x_vals, y_vals, indexing="ij")
    dim_x, dim_y = Z.shape

    vertices = np.column_stack([X.ravel(), Y.ravel(), Z.ravel()])

    triangles = []
    for i in range(dim_x - 1):
        for j in range(dim_y - 1):
            v0 = i * dim_y + j
            v1 = v0 + 1
            v2 = (i + 1) * dim_y + j
            v3 = v2 + 1
            triangles.append([v0, v1, v2])
            triangles.append([v1, v3, v2])

    mesh = o3d.geometry.TriangleMesh()
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    mesh.triangles = o3d.utility.Vector3iVector(np.array(triangles))
    mesh.compute_vertex_normals()

    z_norm = (Z.ravel() - Z.min()) / (Z.max() - Z.min() + 1e-9)
    colors = plt.cm.viridis(z_norm)[:, :3]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    return mesh


def create_sphere(position: np.ndarray, color: list, radius: float = 50.0):
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(position)
    sphere.paint_uniform_color(color)
    sphere.compute_vertex_normals()
    return sphere


def build_ray_lines(ray_path: str, src_pos: np.ndarray) -> o3d.geometry.LineSet:
    rays: pd.DataFrame = readers.read_rays(ray_path, dim=3)
    all_points = []
    all_lines = []
    all_colors = []
    pt_offset = 0

    # Offset to shift source-centered rays into absolute coordinates
    src_offset = np.array([src_pos[0], src_pos[1], 0.0])

    cmap = plt.cm.plasma
    n_rays = len(rays)

    for idx, ray_pts in enumerate(rays.ray):
        pts = np.array(ray_pts)
        n = len(pts)
        if n < 2:
            continue
        all_points.extend(pts.tolist())
        for i in range(n - 1):
            all_lines.append([pt_offset + i, pt_offset + i + 1])
        color = list(cmap(idx / max(n_rays - 1, 1))[:3])
        all_colors.extend([color] * (n - 1))
        pt_offset += n

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(np.array(all_points))
    line_set.lines = o3d.utility.Vector2iVector(np.array(all_lines))
    line_set.colors = o3d.utility.Vector3dVector(np.array(all_colors))
    return line_set


def find_connecting_rays(ray_path: str, rcv_pos: np.ndarray, src_pos: np.ndarray,
                         threshold_m: float = 500.0):
    """Find all rays that pass within threshold of the receiver.
    Returns list of (index, min_dist, ray_pts, angle), sorted by distance."""
    rays: pd.DataFrame = readers.read_rays(ray_path, dim=3)
    hits = []

    for idx, ray_pts in enumerate(rays.ray):
        pts = np.array(ray_pts)
        dists = np.linalg.norm(pts - rcv_pos, axis=1)
        min_dist = dists.min()
        if min_dist < threshold_m:
            angle = rays.angle_of_departure.iloc[idx]
            hits.append((idx, min_dist, pts, angle))

    hits.sort(key=lambda x: x[1])
    return hits


def read_arrivals_3d(arr_path: str, true_range: float = 0.0) -> pd.DataFrame:
    """Read 3D bellhop arrivals file."""
    def _read_pair(f):
        """Read a line with 'count value' format."""
        parts = f.readline().split()
        return int(parts[0]), float(parts[1])

    with open(arr_path) as f:
        f.readline()  # '3D'
        freq = float(f.readline().strip())
        _n_sx, sx = _read_pair(f)
        _n_sy, sy = _read_pair(f)
        _n_sz, sz = _read_pair(f)
        _n_rz, rz = _read_pair(f)
        _n_rr, rr = _read_pair(f)
        _n_th, th = _read_pair(f)

        n_arrivals = int(f.readline().strip())
        # Skip duplicate count line
        f.readline()

        arrivals = []
        for _ in range(n_arrivals):
            parts = f.readline().split()
            arrivals.append({
                'amplitude': float(parts[0]),
                'phase_deg': float(parts[1]),
                'travel_time': float(parts[2]),
                'imag_travel_time': float(parts[3]),
                'departure_elev': float(parts[4]),
                'arrival_elev': float(parts[5]),
                'departure_bearing': float(parts[6]),
                'arrival_bearing': float(parts[7]),
                'surface_bounces': int(parts[8]),
                'bottom_bounces': int(parts[9]),
            })

    df = pd.DataFrame(arrivals)
    df['range_m'] = df['travel_time'] * 1500.0
    df['true_range_m'] = true_range
    df['range_error_m'] = df['range_m'] - true_range
    df['range_error_pct'] = (df['range_error_m'] / true_range * 100.0) if true_range > 0 else 0.0
    return df


def _val(line: str) -> str:
    """Strip comment (!) and whitespace from an env file line."""
    return line.split("!")[0].strip()


def parse_env(env_path: str):
    """Parse source/receiver positions and beam fan from a 3D .env file.

    Fixed format (0-indexed line numbers):
      7:  NSX              8:  SX (km)
      9:  NSY              10: SY (km)
      11: NSD              12: SD (m)
      13: NRD              14: RD (m)
      15: NR               16: R (km)
      17: Ntheta           18: bearing (deg)
      19: RunType          20: Nalpha
      21: alpha_min alpha_max /   (elevation, degrees)
      22: Nbeta            23: beta_min beta_max /  (azimuth, degrees)

    Returns:
        src_pos:     Source position [x, y, z] in meters.
        rcv_pos:     Receiver position [x, y, z] in meters.
        alpha_range: (alpha_min, alpha_max) elevation angles in degrees.
        beta_range:  (beta_min, beta_max) azimuth angles in degrees.
    """
    with open(env_path) as f:
        lines = f.readlines()

    sx = float(_val(lines[8]).split()[0]) * 1000.0    # km → m
    sy = float(_val(lines[10]).split()[0]) * 1000.0   # km → m
    sd = float(_val(lines[12]).split()[0])             # m
    rd = float(_val(lines[14]).split()[0])             # m
    rcv_r = float(_val(lines[16]).split()[0]) * 1000.0  # km → m
    bearing = float(_val(lines[18]).split()[0])         # deg

    src_pos = np.array([sx, sy, sd])

    # Bearing was computed in C++ as atan2(dy, dx), so:
    #   dx = R * cos(bearing), dy = R * sin(bearing)
    bearing_rad = np.radians(bearing)
    rcv_x = sx + rcv_r * np.cos(bearing_rad)
    rcv_y = sy + rcv_r * np.sin(bearing_rad)
    rcv_pos = np.array([rcv_x, rcv_y, rd])

    # Beam fan angles
    alpha_parts = _val(lines[21]).replace("/", "").split()
    alpha_range = (float(alpha_parts[0]), float(alpha_parts[1]))

    beta_parts = _val(lines[23]).replace("/", "").split()
    beta_range = (float(beta_parts[0]), float(beta_parts[1]))

    return src_pos, rcv_pos, alpha_range, beta_range


def build_fan_center_arrow(src_pos: np.ndarray,
                           alpha_range: tuple[float, float],
                           beta_range: tuple[float, float],
                           length: float = 2000.0) -> list[o3d.geometry.Geometry]:
    """Build a yellow arrow showing the beam fan center direction.

    Direction is computed from the midpoint of the alpha (elevation) and
    beta (azimuth) angle ranges using Bellhop3D conventions:
      - alpha: elevation in degrees, positive = downward
      - beta:  azimuth in degrees, math convention (0° = +x)

    Args:
        src_pos:     Source position [x, y, z] in meters.
        alpha_range: (alpha_min, alpha_max) elevation in degrees.
        beta_range:  (beta_min, beta_max) azimuth in degrees.
        length:      Arrow length in meters.
    """
    center_elev = np.radians(np.mean(alpha_range))
    center_bear = np.radians(np.mean(beta_range))

    # Spherical → Cartesian (math convention, positive elev = downward)
    direction = np.array([
        np.cos(center_elev) * np.cos(center_bear),
        np.cos(center_elev) * np.sin(center_bear),
        np.sin(center_elev),
    ])

    print(f"Fan center: elev={np.degrees(center_elev):.2f}°, "
          f"bear={np.degrees(center_bear):.2f}°, "
          f"dir=[{direction[0]:.4f}, {direction[1]:.4f}, {direction[2]:.4f}]")

    tip = src_pos + direction * length
    cone_base = src_pos + direction * (length * 0.85)

    # Shaft as a cylinder
    shaft = o3d.geometry.TriangleMesh.create_cylinder(
        radius=10.0, height=length * 0.85)
    shaft.compute_vertex_normals()

    # Cone head
    cone = o3d.geometry.TriangleMesh.create_cone(
        radius=30.0, height=length * 0.15)
    cone.compute_vertex_normals()

    # Both default along +Z centered at origin. Rotate to target direction.
    z_axis = np.array([0.0, 0.0, 1.0])
    v = np.cross(z_axis, direction)
    c = np.dot(z_axis, direction)
    if np.linalg.norm(v) < 1e-8:
        R = np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        vx = np.array([[0, -v[2], v[1]],
                        [v[2], 0, -v[0]],
                        [-v[1], v[0], 0]])
        R = np.eye(3) + vx + vx @ vx / (1.0 + c)

    # Cylinder is centered at origin along Z, so translate its midpoint
    shaft.rotate(R, center=[0, 0, 0])
    shaft.translate(src_pos + direction * (length * 0.85 / 2.0))

    # Cone base is at z=0 in its local frame
    cone.rotate(R, center=[0, 0, 0])
    cone.translate(cone_base)

    shaft.paint_uniform_color([1.0, 1.0, 0.0])
    cone.paint_uniform_color([1.0, 1.0, 0.0])

    return [shaft, cone]


if __name__ == "__main__":
    geometries = []

    bty_path = file_root + ".bty"
    env_path = file_root + ".env"
    ray_path = file_root + ".ray"

    # Bathymetry mesh
    if os.path.exists(bty_path):
        bty = read_bty_3d(bty_path)
        print(f"Bathy X (range)  : [{bty['ranges'][0]:.1f}, {bty['ranges'][-1]:.1f}] m  ({len(bty['ranges'])} pts)")
        print(f"Bathy Y (xrange) : [{bty['crossranges'][0]:.1f}, {bty['crossranges'][-1]:.1f}] m  ({len(bty['crossranges'])} pts)")
        print(f"Bathy depth      : [{bty['depths'].min():.1f}, {bty['depths'].max():.1f}] m  shape={bty['depths'].shape}")
        mesh = build_bathy_mesh(bty_path)
        geometries.append(mesh)
        print(f"Loaded bathymetry: {bty_path}")
    else:
        print(f"No bathymetry file: {bty_path}")

    # Source and receiver markers + beam fan center
    alpha_range = None
    beta_range = None
    if os.path.exists(env_path):
        src_pos, rcv_pos, alpha_range, beta_range = parse_env(env_path)
        print(f"Source:   {src_pos}")
        print(f"Receiver: {rcv_pos}")
        print(f"Alpha (elevation): {alpha_range[0]:.2f}° to {alpha_range[1]:.2f}°")
        print(f"Beta  (azimuth):   {beta_range[0]:.2f}° to {beta_range[1]:.2f}°")
        geometries.append(create_sphere(src_pos, [1, 0, 0], radius=50.0))  # red
        geometries.append(create_sphere(rcv_pos, [0, 1, 0], radius=50.0))  # green
        geometries.extend(build_fan_center_arrow(
            src_pos, alpha_range, beta_range))
    else:
        print(f"No env file: {env_path}")

    # Arrivals info
    arr_path = file_root + ".arr"
    if os.path.exists(arr_path) and os.path.exists(env_path):
        true_range = np.linalg.norm(rcv_pos - src_pos)
        arr = read_arrivals_3d(arr_path, true_range)
        print(f"\n--- {len(arr)} arrivals (true range: {true_range:.1f}m) ---")
        print(arr[['travel_time', 'range_m', 'range_error_m', 'range_error_pct',
                    'amplitude', 'departure_elev', 'departure_bearing',
                    'surface_bounces', 'bottom_bounces']].to_string(index=False))
        if len(arr) > 0:
            best = arr.loc[arr['travel_time'].idxmin()]
            print(f"\nFirst arrival: TOF={best['travel_time']:.6f}s, "
                  f"range~{best['range_m']:.1f}m, "
                  f"error={best['range_error_m']:.1f}m ({best['range_error_pct']:.1f}%), "
                  f"bounces(sfc={int(best['surface_bounces'])}, "
                  f"bot={int(best['bottom_bounces'])})")
    else:
        print(f"No arrivals file: {arr_path}")

    # Ray traces (optional)
    connecting = []
    if os.path.exists(ray_path):
        line_set = build_ray_lines(ray_path, src_pos)
        geometries.append(line_set)
        print(f"Loaded rays: {ray_path}")

        if os.path.exists(env_path):
            threshold_m = 50
            connecting = find_connecting_rays(ray_path, rcv_pos, src_pos, threshold_m=threshold_m)
            print(f"\n--- {len(connecting)} rays within {threshold_m}m of receiver ---")
            for i, (idx, dist, pts, angle) in enumerate(connecting):
                label = "BEST" if i == 0 else f"  #{i+1}"
                print(f"  {label}: ray {idx}, departure={angle:.2f}°, "
                      f"closest approach={dist:.2f}m")
    else:
        print(f"No ray file (run bhc_runner to generate): {ray_path}")

    # Coordinate frame
    geometries.append(
        o3d.geometry.TriangleMesh.create_coordinate_frame(size=100)
    )

    if not geometries:
        print("No files found to visualize")
        exit(1)

    # Window 1: All rays + bathymetry
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="All Rays", width=960, height=720)
    for g in geometries:
        vis.add_geometry(g)
    ctr = vis.get_view_control()
    ctr.set_front([0, 0, -1])
    ctr.set_up([0, 1, 0])

    # Window 2: Only connecting rays + source/receiver
    if connecting:
        vis2 = o3d.visualization.Visualizer()
        vis2.create_window(window_name="Connecting Rays", width=960, height=720,
                           left=980)
        if os.path.exists(bty_path):
            vis2.add_geometry(build_bathy_mesh(bty_path))
        vis2.add_geometry(create_sphere(src_pos, [1, 0, 0], radius=50.0))
        vis2.add_geometry(create_sphere(rcv_pos, [0, 1, 0], radius=50.0))

        cmap = plt.cm.plasma
        for i, (idx, dist, pts, angle) in enumerate(connecting):
            color = list(cmap(i / max(len(connecting) - 1, 1))[:3])
            n = len(pts)
            lines = [[j, j + 1] for j in range(n - 1)]
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(pts)
            ls.lines = o3d.utility.Vector2iVector(np.array(lines))
            ls.colors = o3d.utility.Vector3dVector(np.array([color] * (n - 1)))
            vis2.add_geometry(ls)

        if alpha_range is not None and beta_range is not None:
            for g in build_fan_center_arrow(src_pos, alpha_range, beta_range):
                vis2.add_geometry(g)
        vis2.add_geometry(o3d.geometry.TriangleMesh.create_coordinate_frame(size=100))
        ctr2 = vis2.get_view_control()
        ctr2.set_front([0, 0, -1])
        ctr2.set_up([0, 1, 0])

        # Run both windows
        while vis.poll_events() and vis2.poll_events():
            vis.update_renderer()
            vis2.update_renderer()
        vis.destroy_window()
        vis2.destroy_window()
    else:
        vis.run()
        vis.destroy_window()
