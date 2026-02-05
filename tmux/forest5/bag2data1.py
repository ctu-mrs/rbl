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
DISTANCE_THRESHOLD = 0.4  # meters
MAX_SYNC_DT = 0.05        # seconds for nearest-time sync
TRAJ_CMAP = 'viridis'



# ===============================
# Load bag positions by topic
# ===============================
def load_bag_positions(bag_path, uav_topics):
    """
    Returns a dict: {topic_name: (timestamps, positions)}
    timestamps in seconds (bag time)
    positions: Nx3 numpy array
    """
    data = defaultdict(list)

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=uav_topics):
            pos = msg.pose.pose.position
            data[topic].append((t.to_sec(), np.array([pos.x, pos.y, pos.z])))

    out = {}
    for topic, samples in data.items():
        samples.sort(key=lambda x: x[0])  # ensure monotonic
        t = np.array([s[0] for s in samples])
        p = np.array([s[1] for s in samples])
        keep = np.concatenate([[True], np.diff(t) > 0])  # drop duplicates
        out[topic] = (t[keep], p[keep])
    return out

def bag_to_dataframe(bag_file, uav_topics):
    """
    Convert rosbag to pandas DataFrame with columns:
    field.child_frame_id, field.header.stamp, field.pose.pose.position.x/y/z, field.twist.twist.linear.x/y/z
    """
    rows = []

    with rosbag.Bag(bag_file) as bag:
        for topic, msg, t in bag.read_messages(topics=uav_topics):
            row = {
                "field.child_frame_id": topic.split('/')[1],  # uav1, uav2, etc.
                "field.header.stamp": t.to_sec(),
                "field.pose.pose.position.x": msg.pose.pose.position.x,
                "field.pose.pose.position.y": msg.pose.pose.position.y,
                "field.pose.pose.position.z": msg.pose.pose.position.z,
                "field.twist.twist.linear.x": msg.twist.twist.linear.x,
                "field.twist.twist.linear.y": msg.twist.twist.linear.y,
                "field.twist.twist.linear.z": msg.twist.twist.linear.z,
            }
            rows.append(row)

    df = pd.DataFrame(rows)
    return df

# ===============================
# Load ASCII PCD
# ===============================
def load_pcd_ascii(filename):
    points = []
    with open(filename) as f:
        data = False
        for line in f:
            if line.startswith("DATA"):
                if "ascii" not in line:
                    raise ValueError("Only ASCII PCD supported")
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
# Compute speed from positions
# ===============================
def compute_speeds(p, dt=0.01):
    """
    Simple finite-difference speed approximation
    """
    v = np.diff(p, axis=0) / dt
    speeds = np.linalg.norm(v, axis=1)
    speeds = np.concatenate([[speeds[0]], speeds])  # keep length same
    return speeds

def calculate_speed(df):
    """Calculate speed magnitude for each data point"""
    vx = df["field.twist.twist.linear.x"].values
    vy = df["field.twist.twist.linear.y"].values
    vz = df["field.twist.twist.linear.z"].values
    return np.sqrt(vx**2 + vy**2 + vz**2)

# ===============================
# Plot UAV trajectories 3D with speed coloring
# ===============================

def plot_trajectories_and_obstacles_3d(df, obstacle_points, cmap='viridis'):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')
    
    groups = df.groupby("field.child_frame_id")
    
    # Collect all speeds for consistent color mapping
    all_speeds = []
    for _, g in groups:
        speeds = calculate_speed(g)
        all_speeds.extend(speeds)
    
    # Create color normalization based on all speeds
    speed_min, speed_max = np.min(all_speeds), np.max(all_speeds)
    norm = Normalize(vmin=speed_min, vmax=speed_max)
    cmap_obj = plt.cm.get_cmap(cmap)
    
    # --- UAV trajectories with velocity coloring ---
    for i, (uav, g) in enumerate(groups):
        x = g["field.pose.pose.position.x"].values
        y = g["field.pose.pose.position.y"].values
        z = g["field.pose.pose.position.z"].values
        speeds = calculate_speed(g)
        
        # Create a line collection with varying colors
        for j in range(len(x) - 1):
            # Average speed between points for color
            avg_speed = (speeds[j] + speeds[j + 1]) / 2
            color = cmap_obj(norm(avg_speed))
            
            ax.plot(
                x[j:j+2],
                y[j:j+2],
                z[j:j+2],
                color=color,
                linewidth=2
            )
    
    # --- Obstacle point cloud ---
    ax.scatter(
        obstacle_points[:, 0],
        obstacle_points[:, 1],
        obstacle_points[:, 2],
        s=1,
        c='gray',
        alpha=0.1
    )
    
    # --- Add colorbar ---
    sm = ScalarMappable(cmap=cmap_obj, norm=norm)
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, pad=0.1, shrink=0.5)
    cbar.set_label('Speed (m/s)', fontsize=fontsize)
    
    # --- Axes ---
    ax.set_xlabel("x (m)", fontsize=fontsize)
    ax.set_ylabel("y (m)", fontsize=fontsize)
    ax.set_zlabel("z (m)", fontsize=fontsize)
    
    # ax.set_box_aspect([1, 1, 0.25])
    
    # Add a simple legend for UAV identification (optional)
    # You might want to show one colored segment per UAV in the legend
    handles = []
    labels = []
    for i, (uav, _) in enumerate(groups):
        # Use a fixed color from tab10 for UAV identification
        color = plt.cm.tab10(i % 10)
        handles.append(plt.Line2D([0], [0], color=color, linewidth=2))
        labels.append(uav)
    
    # if len(handles) > 0:
        # ax.legend(handles, labels, loc='upper right')

# ===============================
# Plot pairwise distances over time
# ===============================
def plot_pairwise_distances(data):
    topics = list(data.keys())
    fig, ax = plt.subplots(figsize=(20, 6))

    for topic_i, topic_j in itertools.combinations(topics, 2):
        t_i, p_i = data[topic_i]
        t_j, p_j = data[topic_j]

        dists = []
        t_plot = []

        for k, ti in enumerate(t_i):
            idx = np.argmin(np.abs(t_j - ti))
            dt = abs(t_j[idx] - ti)
            if dt > MAX_SYNC_DT:
                continue
            d = np.linalg.norm(p_i[k] - p_j[idx])
            dists.append(d)
            t_plot.append(ti - t_i[0])

        ax.plot(t_plot, dists, linewidth=1, label=f"{topic_i} ↔ {topic_j}")

    ax.axhline(DISTANCE_THRESHOLD, linestyle="--", color="magenta",
               label="collision threshold")

    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (m)")
    ax.set_title("Pairwise Inter-UAV Distances")
    ax.grid(True)
    # ax.legend(fontsize=8, ncol=2)
    plt.tight_layout()



def compute_min_pairwise_distance(data, max_sync_dt=float('inf')):
    """
    Computes the minimum distance between all pairs of topics (UAVs).
    
    Args:
        data: dict of {topic_name: (time_array, position_array)}
        max_sync_dt: maximum allowed time difference to compare positions
        
    Returns:
        min_distance: float, minimum distance across all pairs
        min_info: tuple (topic_i, topic_j, time) where min occurs
    """
    topics = list(data.keys())
    min_distance = float('inf')
    min_info = None

    for topic_i, topic_j in itertools.combinations(topics, 2):
        t_i, p_i = data[topic_i]
        t_j, p_j = data[topic_j]

        for k, ti in enumerate(t_i):
            idx = np.argmin(np.abs(t_j - ti))
            dt = abs(t_j[idx] - ti)
            if dt > max_sync_dt:
                continue

            d = np.linalg.norm(p_i[k] - p_j[idx])
            if d < min_distance:
                min_distance = d
                min_info = (topic_i, topic_j, ti)

    return min_distance, min_info
# ===============================
# Main
# ===============================
def main(bag_file,pcd_file):
    # List UAV topics automatically

    with rosbag.Bag(bag_file) as bag:
        topics = [t for t, _, _ in bag.read_messages() if 'odom_main' in t]
        topics = sorted(set(topics))  # remove duplicates

    print("Detected UAV topics:", topics)
    data = load_bag_positions(bag_file, topics)
    df =  bag_to_dataframe(bag_file,topics)

    obstacle_points = load_pcd_ascii(pcd_file)

    # plot_trajectories_and_obstacles_3d(df, obstacle_points)
    # plot_trajectories_3d(data)
    plot_pairwise_distances(data)


    min_dist, info = compute_min_pairwise_distance(data, max_sync_dt=0.1)
    print(f"Minimum distance: {min_dist:.3f} m between {info[0]} and {info[1]} at t={info[2]:.2f}s")
    plt.show()


# ===============================
# Entry point
# ===============================
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot UAV trajectories and distances from rosbag")
    parser.add_argument("bag_file", help="Path to rosbag")
    parser.add_argument("pcd_file", help="Obstacle PCD (ASCII)")
    args = parser.parse_args()

    main(args.bag_file, args.pcd_file)
