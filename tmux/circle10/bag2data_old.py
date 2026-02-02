#!/usr/bin/env python3

import rosbag
import numpy as np
import matplotlib.pyplot as plt
import itertools
from collections import defaultdict

# ---------------- USER SETTINGS ----------------

BAG_PATH = "rbl_odom_2.bag"

UAV_TOPICS = [
    "/uav1/estimation_manager/odom_main",
    "/uav2/estimation_manager/odom_main",
    "/uav3/estimation_manager/odom_main",
    "/uav4/estimation_manager/odom_main",
    "/uav5/estimation_manager/odom_main",
    "/uav6/estimation_manager/odom_main",
    "/uav7/estimation_manager/odom_main",
    "/uav8/estimation_manager/odom_main",
    "/uav9/estimation_manager/odom_main",
    "/uav10/estimation_manager/odom_main",
]

DISTANCE_THRESHOLD = 1.0      # meters
MAX_SYNC_DT = 0.05            # seconds (time sync tolerance)

# ------------------------------------------------


def load_bag_positions(bag_path, topics):
    """
    Load positions indexed by BAG TIME (rosbag timestamp).
    """
    data = defaultdict(list)

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            pos = msg.pose.pose.position
            data[topic].append((
                t.to_sec(),                      # <-- BAG TIME
                np.array([pos.x, pos.y, pos.z])
            ))

    # convert to numpy arrays
    out = {}
    for topic, samples in data.items():
        samples.sort(key=lambda x: x[0])  # ensure time order
        t = np.array([s[0] for s in samples])
        p = np.array([s[1] for s in samples])

        # drop duplicate timestamps (VERY IMPORTANT)
        keep = np.concatenate([[True], np.diff(t) > 0])
        out[topic] = (t[keep], p[keep])

    return out


def plot_all_pairwise_distances(data):
    topics = list(data.keys())
    fig, ax = plt.subplots(figsize=(20, 6))

    # Iterate over all unique UAV pairs
    for topic_i, topic_j in itertools.combinations(topics, 2):
        t_i, p_i = data[topic_i]
        t_j, p_j = data[topic_j]

        # compute distances at nearest times
        dists = []
        t_plot = []

        for k, ti in enumerate(t_i):
            idx = np.argmin(np.abs(t_j - ti))
            dt = abs(t_j[idx] - ti)

            if dt > MAX_SYNC_DT:
                continue  # skip if not synchronized

            d = np.linalg.norm(p_i[k] - p_j[idx])
            dists.append(d)
            t_plot.append(ti - t_i[0])  # start from zero

        ax.plot(t_plot, dists, linewidth=1, label=f"{topic_i} ↔ {topic_j}")

    ax.axhline(DISTANCE_THRESHOLD, linestyle="--", color="magenta",
               label="collision threshold")

    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (m)")
    ax.grid(True)
    ax.legend(ncol=2, fontsize=8)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    data = load_bag_positions(BAG_PATH, UAV_TOPICS)

    # sanity check (must pass)
    for topic, (t, _) in data.items():
        dt_min = np.min(np.diff(t))
        if dt_min <= 0:
            raise RuntimeError(f"Non-monotonic time in {topic}")

    plot_all_pairwise_distances(data)
