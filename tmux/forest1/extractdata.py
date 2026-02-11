#!/usr/bin/env python3


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
fontsize = 12
plt.rcParams['mathtext.fontset'] = 'stix'
plt.rcParams['font.family'] = 'serif'
plt.rcParams.update({'pdf.fonttype': 42})
# ===============================
# Settings
# ===============================
VEL_START_THRESHOLD = 0.2    # m/s (XY)
GOAL_THRESHOLD = 1.5         # m
DISTANCE_THRESHOLD = 0.4     # m
MAX_SYNC_DT = 0.1            # s
TRAJ_CMAP = 'viridis'

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
        final_pos = g[["x", "y", "z"]].values[-1]
        arrived = False

        for _, row in g.iterrows():
            pos = np.array([row["x"], row["y"], row["z"]])
            if np.linalg.norm(pos - final_pos) < GOAL_THRESHOLD:
                arrival_times[uav] = row["time"]
                arrived = True
                break

        if not arrived:
            raise RuntimeError(f"{uav} never reached goal")

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


def extract_metrics(df_cut):
    metrics = {}
    for uav, g in df_cut.groupby("uav"):
        metrics[uav] = {
            "path_length": path_length_xyz(g["x"].values,
                                           g["y"].values,
                                           g["z"].values),
            "avg_velocity": avg_velocity_xyz(g)
        }
    return metrics

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
    ax.plot_surface(x, y, z, color=color, linewidth=0, shade=True)

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


def plot_trajectories_3d(df, obstacle_points):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=60, azim=60)

    groups = df.groupby("uav")
    speeds = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
    norm = Normalize(vmin=speeds.min(), vmax=speeds.max())
    cmap = plt.cm.get_cmap(TRAJ_CMAP)

    ROBOT_RADIUS = 0.20  # meters (footprint radius)

    STEP = 1  # plot sphere every 2 points for speed

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

    ax.scatter(obstacle_points[:, 0],
               obstacle_points[:, 1],
               obstacle_points[:, 2],
               s=1, c='gray', alpha=0.05)

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

    plt.savefig("trajectories_3d.pdf", format="pdf", bbox_inches="tight")
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

    for uav, m in metrics.items():
        print(f"{uav}: path={m['path_length']:.2f} m, avg v={m['avg_velocity']:.2f} m/s")

    print(f"\nMean path length: {np.mean(lengths):.2f} ± {np.std(lengths):.2f}")
    print(f"Mean avg velocity: {np.mean(vels):.2f} ± {np.std(vels):.2f}")

    obstacle_points = load_pcd_ascii(pcd_file)
    plot_trajectories_3d(df_cut, obstacle_points)

    min_dist, info = compute_min_pairwise_distance(data)
    print(f"Min distance: {min_dist:.3f} m between {info[0]} and {info[1]}")

# ===============================
# Entry point
# ===============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file")
    parser.add_argument("pcd_file")
    args = parser.parse_args()
    main(args.bag_file, args.pcd_file)
# import argparse
# import itertools
# import numpy as np
# import rosbag
# import pandas as pd
# import matplotlib.pyplot as plt

# from collections import defaultdict
# from matplotlib.colors import Normalize
# from matplotlib.cm import ScalarMappable
# from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

# # ===============================
# # Settings
# # ===============================
# VEL_START_THRESHOLD = 0.2   # m/s (XYZ)
# GOAL_THRESHOLD = 1.5         # m
# DISTANCE_THRESHOLD = 0.4     # m
# MAX_SYNC_DT = 0.1           # s
# TRAJ_CMAP = 'viridis'

# # ===============================
# # Load bag data
# # ===============================
# def load_bag_positions(bag_path, topics):
#     data = defaultdict(list)

#     with rosbag.Bag(bag_path) as bag:
#         for topic, msg, t in bag.read_messages(topics=topics):
#             p = msg.pose.pose.position
#             data[topic].append((t.to_sec(), np.array([p.x, p.y, p.z])))

#     out = {}
#     for topic, samples in data.items():
#         samples.sort(key=lambda x: x[0])
#         t = np.array([s[0] for s in samples])
#         p = np.array([s[1] for s in samples])
#         keep = np.concatenate([[True], np.diff(t) > 0])
#         out[topic] = (t[keep], p[keep])

#     return out


# def bag_to_dataframe(bag_file, topics):
#     rows = []

#     with rosbag.Bag(bag_file) as bag:
#         for topic, msg, t in bag.read_messages(topics=topics):
#             rows.append({
#                 "uav": topic.split('/')[1],
#                 "time": t.to_sec(),
#                 "x": msg.pose.pose.position.x,
#                 "y": msg.pose.pose.position.y,
#                 "z": msg.pose.pose.position.z,
#                 "vx": msg.twist.twist.linear.x,
#                 "vy": msg.twist.twist.linear.y,
#                 "vz": msg.twist.twist.linear.z,
#             })

#     return pd.DataFrame(rows)

# # ===============================
# # Start / End detection
# # ===============================
# def find_start_end_time(df):
#     groups = {u: g.sort_values("time") for u, g in df.groupby("uav")}

#     final_pos = {
#         u: g[["x", "y", "z"]].values[-1]
#         for u, g in groups.items()
#     }

#     common_times = sorted(set.intersection(*[
#         set(g["time"]) for g in groups.values()
#     ]))

# # --- start time (XY velocity only) ---
#     start_time = None
#     for t in common_times:
#         if all(
#             np.hypot(
#                 g[g["time"] == t]["vx"].values[0],
#                 g[g["time"] == t]["vy"].values[0]
#             ) > VEL_START_THRESHOLD
#             for g in groups.values()
#         ):
#             start_time = t
#             break

#     # --- end time ---
#     end_time = None
#     for t in common_times:
#         if all(
#             np.linalg.norm(
#                 g[g["time"] == t][["x", "y", "z"]].values[0]
#                 - final_pos[u]
#             ) < GOAL_THRESHOLD
#             for u, g in groups.items()
#         ):
#             end_time = t
#             break

#     if start_time is None:
#         raise RuntimeError("Start time not found")
#     if end_time is None:
#         raise RuntimeError("End time not found")

#     return start_time, end_time


# def cut_dataframe(df, start_t, end_t):
#     return df[(df["time"] >= start_t) & (df["time"] <= end_t)].copy()

# # ===============================
# # Metrics
# # ===============================
# def path_length_xyz(x, y, z):
#     dp = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)
#     return np.sum(dp)


# def avg_velocity_xyz(df):
#     v = np.sqrt(df["vx"]**2 + df["vy"]**2 + df["vz"]**2)
#     return np.mean(v)


# def extract_metrics(df_cut):
#     metrics = {}

#     for uav, g in df_cut.groupby("uav"):
#         length = path_length_xyz(g["x"].values,
#                                  g["y"].values,
#                                  g["z"].values)
#         v_avg = avg_velocity_xyz(g)

#         metrics[uav] = {
#             "path_length": length,
#             "avg_velocity": v_avg
#         }

#     return metrics

# # ===============================
# # Obstacles
# # ===============================
# def load_pcd_ascii(filename):
#     points = []
#     with open(filename) as f:
#         data = False
#         for line in f:
#             if line.startswith("DATA"):
#                 data = True
#                 continue
#             if data:
#                 vals = line.split()
#                 if len(vals) >= 3:
#                     points.append([float(vals[0]),
#                                    float(vals[1]),
#                                    float(vals[2])])
#     return np.asarray(points)

# # ===============================
# # Plotting
# # ===============================
# def plot_trajectories_3d(df, obstacle_points):
#     fig = plt.figure(figsize=(12, 9))
#     ax = fig.add_subplot(111, projection='3d')

#     groups = df.groupby("uav")
#     speeds_all = []

#     for _, g in groups:
#         speeds_all.extend(np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2))

#     norm = Normalize(vmin=min(speeds_all), vmax=max(speeds_all))
#     cmap = plt.cm.get_cmap(TRAJ_CMAP)

#     for uav, g in groups:
#         x, y, z = g["x"].values, g["y"].values, g["z"].values
#         v = np.sqrt(g["vx"]**2 + g["vy"]**2 + g["vz"]**2)

#         for i in range(len(x) - 1):
#             color = cmap(norm((v.iloc[i] + v.iloc[i+1]) / 2))
#             ax.plot(x[i:i+2], y[i:i+2], z[i:i+2], color=color, linewidth=2)

#     ax.scatter(obstacle_points[:, 0],
#                obstacle_points[:, 1],
#                obstacle_points[:, 2],
#                s=1, c='gray', alpha=0.1)

#     sm = ScalarMappable(norm=norm, cmap=cmap)
#     sm.set_array([])
#     plt.colorbar(sm, ax=ax, label="Speed (m/s)")

#     ax.set_xlabel("x (m)")
#     ax.set_ylabel("y (m)")
#     ax.set_zlabel("z (m)")
#     plt.tight_layout()


# def compute_min_pairwise_distance(data):
#     topics = list(data.keys())
#     min_dist = float('inf')
#     min_info = None

#     for t1, t2 in itertools.combinations(topics, 2):
#         time1, pos1 = data[t1]
#         time2, pos2 = data[t2]

#         for i, ti in enumerate(time1):
#             j = np.argmin(np.abs(time2 - ti))
#             if abs(time2[j] - ti) > MAX_SYNC_DT:
#                 continue
#             d = np.linalg.norm(pos1[i] - pos2[j])
#             if d < min_dist:
#                 min_dist = d
#                 min_info = (t1, t2, ti)

#     return min_dist, min_info

# # ===============================
# # Main
# # ===============================
# def main(bag_file, pcd_file):

#     with rosbag.Bag(bag_file) as bag:
#         topics = sorted(set(
#             t for t, _, _ in bag.read_messages() if "odom_main" in t
#         ))

#     print("Detected topics:", topics)

#     data = load_bag_positions(bag_file, topics)
#     df = bag_to_dataframe(bag_file, topics)

#     start_t, end_t = find_start_end_time(df)
#     df_cut = cut_dataframe(df, start_t, end_t)

#     print(f"\nStart time: {start_t:.2f}s")
#     print(f"End time:   {end_t:.2f}s")
#     print(f"Duration:   {end_t - start_t:.2f}s")

#     metrics = extract_metrics(df_cut)

#     lengths = []
#     velocities = []

#     print("\nPer-UAV metrics:")
#     for uav, m in metrics.items():
#         lengths.append(m["path_length"])
#         velocities.append(m["avg_velocity"])
#         print(f"{uav}: "
#               f"path = {m['path_length']:.2f} m, "
#               f"avg v = {m['avg_velocity']:.2f} m/s")

#     print("\nAggregate metrics:")
#     print(f"Mean path length: {np.mean(lengths):.2f} ± {np.std(lengths):.2f}")
#     print(f"Mean avg velocity: {np.mean(velocities):.2f} ± {np.std(velocities):.2f}")

#     obstacle_points = load_pcd_ascii(pcd_file)
#     plot_trajectories_3d(df_cut, obstacle_points)

#     min_dist, info = compute_min_pairwise_distance(data)
#     print(f"\nMinimum distance: {min_dist:.3f} m "
#           f"between {info[0]} and {info[1]} at t={info[2]:.2f}s")

#     # plt.show()

# # ===============================
# # Entry point
# # ===============================
# if __name__ == "__main__":
#     parser = argparse.ArgumentParser()
#     parser.add_argument("bag_file", help="Path to rosbag")
#     parser.add_argument("pcd_file", help="Obstacle PCD (ASCII)")
#     args = parser.parse_args()

#     main(args.bag_file, args.pcd_file)
