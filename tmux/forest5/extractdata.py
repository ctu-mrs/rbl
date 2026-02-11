#!/usr/bin/env python3
import plotly.graph_objects as go
import argparse
import itertools
import numpy as np
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.cm import ScalarMappable
from matplotlib.colors import Normalize
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import rosbag
from collections import defaultdict
import pandas as pd
plt.rcParams.update({'font.size': 18})
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.family'] = 'serif'
plt.rcParams.update({'pdf.fonttype': 42})
# ===============================
# Settings
# ===============================
VEL_START_THRESHOLD = 0.5    # m/s (XY)
GOAL_THRESHOLD = 1.5        # m
DISTANCE_THRESHOLD = 0.4     # m
MAX_SYNC_DT = 0.1            # s
TRAJ_CMAP = 'viridis'

import open3d as o3d

def load_pcd_clean(filename, z_min_clearance=0.5, voxel=0.15):

    pcd = o3d.io.read_point_cloud(filename)

    if pcd is None or len(pcd.points) == 0:
        print("[WARN] Empty or unreadable PCD")
        return None

    points = np.asarray(pcd.points)

    # Remove NaN / inf
    mask = np.isfinite(points).all(axis=1)
    points = points[mask]

    if len(points) == 0:
        print("[WARN] All points invalid")
        return None

    # -------- estimate ground level automatically --------
    # take lowest 10% of points as ground candidates
    z_sorted = np.sort(points[:,2])
    ground_est = np.median(z_sorted[:int(0.1*len(z_sorted))])

    # keep only points above clearance
    points = points[points[:,2] > ground_est + z_min_clearance]

    if len(points) == 0:
        print("[WARN] After ground removal no points remain")
        return None

    # -------- voxel downsample (CRITICAL for Plotly + KDTree speed) --------
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(points)

    pcd2 = pcd2.voxel_down_sample(voxel_size=voxel)
    points = np.asarray(pcd2.points)

    print(f"[PCD] ground={ground_est:.2f}m | kept={len(points)} points")

    return points
def load_pcd(filename, min_points=1000, verbose=True):
    """
    Load a PCD safely.
    Skips empty, NaN, corrupted or degenerate point clouds.
    Returns None if unusable.
    """

    try:
        pcd = o3d.io.read_point_cloud(filename)

        # failed to read
        if pcd is None:
            if verbose:
                print(f"[WARN] {filename}: Open3D returned None")
            return None

        points = np.asarray(pcd.points)

        # empty
        if points.size == 0:
            if verbose:
                print(f"[WARN] {filename}: empty point cloud")
            return None

        # remove NaN and Inf
        mask = np.isfinite(points).all(axis=1)
        points = points[mask]

        if len(points) == 0:
            if verbose:
                print(f"[WARN] {filename}: all points invalid (NaN/Inf)")
            return None

        # remove crazy coordinates (corrupted frames sometimes produce 1e8 meters)
        MAX_RANGE = 500.0  # meters
        mask = np.linalg.norm(points, axis=1) < MAX_RANGE
        points = points[mask]

        if len(points) < min_points:
            if verbose:
                print(f"[WARN] {filename}: too few valid points ({len(points)})")
            return None

        # optional: remove duplicates (very common in RTABMAP)
        points = np.unique(points, axis=0)

        return points

    except Exception as e:
        if verbose:
            print(f"[ERROR] Failed to load {filename}: {e}")
        return None
# ===============================
# Load bag data
# ===============================
def load_bag_positions(bag_path, topics):
    data = defaultdict(list)
    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            p = msg.pose.pose.position
            data[topic].append((t.to_sec(), np.array([p.x, p.y, p.z])))

    out = {}
    for topic, samples in data.items():
        samples.sort(key=lambda x: x[0])
        t = np.array([s[0] for s in samples])
        p = np.array([s[1] for s in samples])
        keep = np.concatenate([[True], np.diff(t) > 0])
        out[topic] = (t[keep], p[keep])
    return out


def bag_to_dataframe(bag_file, topics):
    rows = []
    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            rows.append({
                "uav": topic.split('/')[1],
                "time": t.to_sec(),
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
                "vx": msg.twist.twist.linear.x,
                "vy": msg.twist.twist.linear.y,
                "vz": msg.twist.twist.linear.z,
            })
    return pd.DataFrame(rows)

# ===============================
# Start / End detection (FIXED)
# ===============================
def find_start_time_xy(groups):
    ref_uav = list(groups.keys())[0]
    ref_times = groups[ref_uav]["time"].values

    for t in ref_times:
        all_moving = True
        for uav, g in groups.items():
            idx = np.argmin(np.abs(g["time"].values - t))
            dt = abs(g["time"].values[idx] - t)
            if dt > MAX_SYNC_DT:
                all_moving = False
                break

            vx = g.iloc[idx]["vx"]
            vy = g.iloc[idx]["vy"]
            if np.hypot(vx, vy) < VEL_START_THRESHOLD:
                all_moving = False
                break

        if all_moving:
            return t

    return None


def find_end_time(groups):
    arrival_times = {}

    for uav, g in groups.items():
        final_pos = [80,0,3] #g[["x", "y", "z"]].values[-1]
        arrived = False

        for _, row in g.iterrows():
            pos = np.array([row["x"], row["y"], row["z"]])
            if np.linalg.norm(pos - final_pos) < GOAL_THRESHOLD : 
                arrival_times[uav] = row["time"]
                arrived = True
                break

        if not arrived:
            print(f"{uav} never reached goal") 
            arrival_times[uav] = row["time"]
            arrived = True
  
            # raise RuntimeError(f"{uav} never reached goal")

    return max(arrival_times.values())


def cut_dataframe(df, start_t, end_t):
    return df[(df["time"] >= start_t) & (df["time"] <= end_t)].copy()

# ===============================
# Metrics
# ===============================
def path_length_xyz(x, y, z):
    return np.sum(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2))


def avg_velocity_xyz(df):
    return np.mean(np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2))

def time_weighted_avg_velocity(g):
    t = g["time"].values
    v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2).values

    dt = np.diff(t)

    # integrate |v(t)| dt
    integral = np.sum(v[:-1] * dt)

    total_time = t[-1] - t[0]

    return integral / total_time

def extract_metrics(df_cut):
    metrics = {}
    for uav, g in df_cut.groupby("uav"):

        speed = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)

        metrics[uav] = {
            "path_length": path_length_xyz(g["x"].values,
                                           g["y"].values,
                                           g["z"].values),
            "avg_velocity1": time_weighted_avg_velocity(g),
            "avg_velocity": np.mean(speed),
            "max_velocity": np.max(speed),
            "robust_max_velocity": np.percentile(speed, 95)  # IMPORTANT
        }
    return metrics
# def extract_metrics(df_cut):
#     metrics = {}
#     for uav, g in df_cut.groupby("uav"):
#         metrics[uav] = {
#             "path_length": path_length_xyz(g["x"].values,
#                                            g["y"].values,
#                                            g["z"].values),
#             "avg_velocity": avg_velocity_xyz(g)
#         }
#     return metrics

# ===============================
# Obstacles
# ===============================
def load_pcd_ascii(filename):
    points = []
    with open(filename) as f:
        data = False
        for line in f:
            if line.startswith("DATA"):
                data = True
                continue
            if data:
                vals = line.split()
                if len(vals) >= 3:
                    points.append([float(vals[0]),
                                   float(vals[1]),
                                   float(vals[2])])
    return np.asarray(points)

# ===============================
# Plotting
# ===============================

def plot_sphere(ax, center, radius, color, n=10):
    u, v = np.mgrid[0:2*np.pi:n*1j, 0:np.pi:n*1j]
    x = center[0] + radius * np.cos(u) * np.sin(v)
    y = center[1] + radius * np.sin(u) * np.sin(v)
    z = center[2] + radius * np.cos(v)
    ax.plot_surface(x, y, z, color=color, linewidth=0, shade=True, zorder=10)

def plot_tube_segment(ax, p0, p1, radius, color, n=12):
    # direction vector
    v = p1 - p0
    length = np.linalg.norm(v)
    if length == 0:
        return

    v = v / length

    # find two perpendicular vectors
    if abs(v[2]) < 0.9:
        n1 = np.cross(v, [0, 0, 1])
    else:
        n1 = np.cross(v, [0, 1, 0])
    n1 /= np.linalg.norm(n1)
    n2 = np.cross(v, n1)

    theta = np.linspace(0, 2*np.pi, n)
    circle = radius * (np.outer(np.cos(theta), n1) +
                       np.outer(np.sin(theta), n2))

    # create tube surface
    X = np.vstack([p0[0] + circle[:, 0], p1[0] + circle[:, 0]])
    Y = np.vstack([p0[1] + circle[:, 1], p1[1] + circle[:, 1]])
    Z = np.vstack([p0[2] + circle[:, 2], p1[2] + circle[:, 2]])

    ax.plot_surface(X, Y, Z, color=color, linewidth=0, shade=True)

# ===============================
# Plotting with proper depth using Plotly
# ===============================

def plot_trajectories_3d_p(df, obstacle_points, uav_radius=0.7,
                           pdf_file="trajectories_3d.pdf", html_file="trajectories_3d.html"):
    import plotly.graph_objects as go
    import matplotlib

    # Compute UAV velocity
    speeds = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
    norm = matplotlib.colors.Normalize(vmin=speeds.min(), vmax=speeds.max())
    cmap = matplotlib.colormaps.get_cmap("viridis")

    # Compute axes ranges to scale markers
    x_range = df["x"].max() - df["x"].min()
    y_range = df["y"].max() - df["y"].min()
    z_range = df["z"].max() - df["z"].min()
    max_range = max(x_range, y_range, z_range)

    # approximate pixel-to-meter scaling for markers
    marker_size = max(2, uav_radius / max_range * 200)  # tweak 200 if needed

    fig = go.Figure()

    # Plot UAV trajectories
    for uav, g in df.groupby("uav"):
        x, y, z = g["x"].values, g["y"].values, g["z"].values
        v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)
        col = [f'rgb({int(r*255)},{int(g_*255)},{int(b*255)})'
               for r, g_, b, _ in cmap(norm(v))]

        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines+markers',
            line=dict(color='blue', width=2),
            marker=dict(size=marker_size, color=col),
            name=uav
        ))

    # Plot obstacles
    # fig.add_trace(go.Scatter3d(
    #     x=obstacle_points[:,0],
    #     y=obstacle_points[:,1],
    #     z=obstacle_points[:,2],
    #     mode='markers',
    #     marker=dict(size=2, color='gray', opacity=1),
    #     name='Obstacles'
    # ))

    # Colorbar for speed
    fig.add_trace(go.Scatter3d(
        x=[None], y=[None], z=[None],
        mode='markers',
        marker=dict(
            colorscale='Viridis',
            cmin=speeds.min(),
            cmax=speeds.max(),
            color=[speeds.min()],
            size=0.1,
            colorbar=dict(title="Speed (m/s)")
        ),
        showlegend=False
    ))

    # Make the full box visible
    fig.update_layout(
        scene=dict(
            xaxis=dict(title="$x$ (m)", range=[df["x"].min()-0.1*x_range, df["x"].max()+0.1*x_range]),
            yaxis=dict(title="$y$ (m)", range=[df["y"].min()-0.1*y_range, df["y"].max()+0.1*y_range]),
            zaxis=dict(title="$z$ (m)", range=[df["z"].min()-0.1*z_range, df["z"].max()+0.1*z_range]),
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.2, y=1.2, z=0.6)  # similar to matplotlib elev=60°, azim=60°
            )
        ),
        margin=dict(r=0, l=0, b=0, t=0)
    )

    # Save outputs
    fig.write_html(html_file)
    # fig.write_image(pdf_file)
    print(f"Saved interactive HTML: {html_file}")
    print(f"Saved PDF: {pdf_file}")

    fig.show()
def plot_trajectories_3d_pold(df, obstacle_points, pdf_file="trajectories_3d.pdf", html_file="trajectories_3d.html"):
    import plotly.graph_objects as go
    import matplotlib

    # Colormap for velocity
    speeds = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
    norm = matplotlib.colors.Normalize(vmin=speeds.min(), vmax=speeds.max())
    cmap = matplotlib.colormaps.get_cmap("viridis")

    # Convert speeds to RGB for Plotly
    colors = [f'rgb({int(r*255)},{int(g*255)},{int(b*255)})' 
              for r, g, b, _ in cmap(norm(speeds))]

    fig = go.Figure()

    # Plot UAV trajectories
    for uav, g in df.groupby("uav"):
        x, y, z = g["x"].values, g["y"].values, g["z"].values
        v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)
        # map speeds to colors
        col = [f'rgb({int(r*255)},{int(g_*255)},{int(b*255)})' 
               for r, g_, b, _ in cmap(norm(v))]

        fig.add_trace(go.Scatter3d(
            x=x, y=y, z=z,
            mode='lines+markers',
            line=dict(color='blue', width=2),  # line is blue; markers colored by speed
            marker=dict(size=4, color=col),
            name=uav
        ))

    # Obstacles
    # fig.add_trace(go.Scatter3d(
    #     x=obstacle_points[:,0],
    #     y=obstacle_points[:,1],
    #     z=obstacle_points[:,2],
    #     mode='markers',
    #     marker=dict(size=2, color='gray', opacity=0.3),
    #     name='Obstacles'
    # ))

    # Colorbar for speed
    fig.add_trace(go.Scatter3d(
        x=[None], y=[None], z=[None],
        mode='markers',
        marker=dict(
            colorscale='Viridis',
            cmin=speeds.min(),
            cmax=speeds.max(),
            color=[speeds.min()],
            size=0.1,
            colorbar=dict(title="Speed (m/s)")
        ),
        showlegend=False
    ))

    # Camera view similar to Matplotlib elev=60°, azim=60°
    fig.update_layout(
        scene=dict(
            xaxis=dict(title="$x$ (m)", showspikes=False),
            yaxis=dict(title="$y$ (m)", showspikes=False),
            zaxis=dict(title="$z$ (m)", showspikes=False),
            aspectmode='data',
            camera=dict(
                eye=dict(x=1.2, y=1.2, z=1.2)
            )
        ),
        margin=dict(r=0, l=0, b=0, t=0)
    )

    # Save outputs
    fig.write_html(html_file)
    fig.write_image(pdf_file)
    print(f"Saved interactive HTML: {html_file}")
    print(f"Saved PDF: {pdf_file}")

    fig.show()


def plot_trajectories_3d(df, obstacle_points):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=60, azim=60)
    # ax.view_init(elev=90, azim=-90)

    groups = df.groupby("uav")
    speeds = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
    norm = Normalize(vmin=speeds.min(), vmax=speeds.max())
    cmap = plt.cm.get_cmap(TRAJ_CMAP)

    ROBOT_RADIUS = 0.70  # meters (footprint radius)

    STEP = 1  # plot sphere every 2 points for speed

    # ax.scatter(obstacle_points[:,0],
    #            obstacle_points[:,1],
    #            obstacle_points[:,2],
    #            s=1,
    #            c='gray',
    #            alpha=0.28,
    #            depthshade=False,
    #            zorder=1)

    for uav, g in groups:
        x, y, z = g["x"].values, g["y"].values, g["z"].values
        v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)

        for i in range(0, len(x), STEP):
            center = np.array([x[i], y[i], z[i]])
            color = cmap(norm(v.iloc[i]))
            plot_sphere(ax, center, ROBOT_RADIUS, color, n=10)
            
    # for uav, g in groups:
        # x, y, z = g["x"].values, g["y"].values, g["z"].values
        # v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)

        # for i in range(len(x) - 1):
            # p0 = np.array([x[i], y[i], z[i]])
            # p1 = np.array([x[i+1], y[i+1], z[i+1]])

            # color = cmap(norm((v.iloc[i] + v.iloc[i+1]) / 2))

            # # plot_sphere(ax, p0,ROBOT_RADIUS , color, n=32)
            # # plot_sphere(
            # #     ax,
            # #     center=np.array([1, 2, 3]),
            # #     radius=ROBOT_RADIUS,
            # #     color='steelblue',
            # #     n=32
            # # )
            # plot_tube_segment(
            #     ax,
            #     p0,
            #     p1,
            #     radius=ROBOT_RADIUS,
            #     color=color,
            #     n=30  # increase for smoother tubes
            # )
    # for uav, g in groups:
    #     x, y, z = g["x"].values, g["y"].values, g["z"].values
    #     v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)
    #     for i in range(len(x) - 1):
    #         ax.plot(x[i:i+2], y[i:i+2], z[i:i+2],
    #                 color=cmap(norm((v.iloc[i] + v.iloc[i+1]) / 2)),
    #                 linewidth=2)

    # ax.scatter(obstacle_points[:, 0],
    #            obstacle_points[:, 1],
    #            obstacle_points[:, 2],
    #            s=10, c='gray', alpha=0.05)

    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    plt.colorbar(sm, ax=ax, label="Speed (m/s)")
    plt.tight_layout()

    # ax.set_box_aspect([1, 1, 0.25])

    x_range = df["x"].max() - df["x"].min()
    y_range = df["y"].max() - df["y"].min()
    z_range = df["z"].max() - df["z"].min()

    max_range = max(x_range, y_range, z_range / 0.25)  # scale Z by 0.25

    x_mid = 0.5 * (df["x"].max() + df["x"].min())
    y_mid = 0.5 * (df["y"].max() + df["y"].min())
    z_mid = 0.5 * (df["z"].max() + df["z"].min())

    ax.set_xlim(x_mid - max_range/2, x_mid + max_range/2)
    ax.set_ylim(y_mid - max_range/2, y_mid + max_range/2)
    ax.set_zlim(z_mid - max_range*0.25/2, z_mid + max_range*0.25/2)
    ax.set_box_aspect((1, 1, 0.25))
    from matplotlib.ticker import MaxNLocator
    ax.zaxis.set_major_locator(MaxNLocator(2))
    plt.savefig("trajectories_3d_1.pdf", format="pdf")
    plt.show()
# ===============================
# Pairwise distance
# ===============================
def compute_min_pairwise_distance(data):
    topics = list(data.keys())
    min_dist = float('inf')
    min_info = None

    for t1, t2 in itertools.combinations(topics, 2):
        time1, pos1 = data[t1]
        time2, pos2 = data[t2]

        for i, ti in enumerate(time1):
            j = np.argmin(np.abs(time2 - ti))
            if abs(time2[j] - ti) > MAX_SYNC_DT:
                continue
            d = np.linalg.norm(pos1[i] - pos2[j])
            if d < min_dist:
                min_dist = d
                min_info = (t1, t2, ti)

    return min_dist, min_info

# ===============================
# Main
# ===============================
def main(bag_file, pcd_file):

    with rosbag.Bag(bag_file) as bag:
        topics = sorted(set(
            t for t, _, _ in bag.read_messages() if "odom_main" in t
        ))

    print("Detected topics:", topics)

    data = load_bag_positions(bag_file, topics)
    df = bag_to_dataframe(bag_file, topics)
    groups = {u: g.sort_values("time") for u, g in df.groupby("uav")}

    start_t = find_start_time_xy(groups)
    if start_t is None:
        raise RuntimeError("Start time not found")

    end_t = find_end_time(groups)

    df_cut = cut_dataframe(df, start_t, end_t)

    print(f"\nStart time: {start_t:.2f}s")
    print(f"End time:   {end_t:.2f}s")
    print(f"Duration:   {end_t - start_t:.2f}s")

    metrics = extract_metrics(df_cut)

    lengths = [m["path_length"] for m in metrics.values()]
    vels = [m["avg_velocity"] for m in metrics.values()]
    vels1 = [m["avg_velocity1"] for m in metrics.values()]

    # for uav, m in metrics.items():
    #     print(f"{uav}: path={m['path_length']:.2f} m, avg v={m['avg_velocity']:.2f} m/s")

    # print(f"\nMean path length: {np.mean(lengths):.2f} ± {np.std(lengths):.2f}")
    # print(f"Mean avg velocity: {np.mean(vels):.2f} ± {np.std(vels):.2f}")


    for uav, m in metrics.items():
        print(
            f"{uav}: "
            f"path={m['path_length']:.2f} m | "
            f"avg={m['avg_velocity']:.2f} m/s | "
            f"avg1={m['avg_velocity1']:.2f} m/s | "
            f"max={m['max_velocity']:.2f} m/s | "
            f"robust_max={m['robust_max_velocity']:.2f} m/s"
        )
    obstacle_points = load_pcd_ascii(pcd_file)
    # obstacle_points = []
    plot_trajectories_3d(df_cut, obstacle_points)

    # min_dist, info = compute_min_pairwise_distance(data)
    # print(f"Min distance: {min_dist:.3f} m between {info[0]} and {info[1]}")

# ===============================
# Entry point
# ===============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file")
    parser.add_argument("pcd_file")
    args = parser.parse_args()
    main(args.bag_file, args.pcd_file)

