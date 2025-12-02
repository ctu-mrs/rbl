
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter
import csv

# -----------------------------
# Parameters
# -----------------------------
CSV_FILE = "rbl_odom_4.bag.csv"  
OUTPUT_FILE = "uav_metrics_summary4.csv"
DIST_THRESHOLD = 0.2   # Goal threshold [m]
SPEED_THRESHOLD = 0.1  # Minimum speed to start moving [m/s]
WINDOW_LENGTH = 11
POLYORDER = 2

# -----------------------------
# Load CSV and identify UAVs
# -----------------------------
df = pd.read_csv(CSV_FILE)

if 'uav' not in df.columns:
    df['uav'] = df['field.child_frame_id'].str.extract(r'(uav\d+)')

uav_groups = {}
metrics_list = {}

# -----------------------------
# Process each UAV
# -----------------------------
for uav, group in df.groupby('uav'):
    x = group["field.pose.pose.position.x"].values
    y = group["field.pose.pose.position.y"].values
    z = group["field.pose.pose.position.z"].values
    t = group["%time"].values * 1e-9
    t = t - t[0]  # normalize time to start at 0

    vx = group["field.twist.twist.linear.x"].values
    vy = group["field.twist.twist.linear.y"].values
    vz = group["field.twist.twist.linear.z"].values
    speed = np.sqrt(vx**2 + vy**2 + vz**2)

    # -----------------------------
    # Extract moving segment
    # -----------------------------
    idx_start = np.where(speed > SPEED_THRESHOLD)[0][0]
    goal = np.array([x[-1], y[-1], z[-1]])
    dist_to_goal = np.sqrt((x - goal[0])**2 + (y - goal[1])**2 + (z - goal[2])**2)
    idx_end_candidates = np.where(dist_to_goal <= DIST_THRESHOLD)[0]
    idx_end = idx_end_candidates[0] if len(idx_end_candidates) > 0 else len(t)-1

    x_m = x[idx_start:idx_end+1]
    y_m = y[idx_start:idx_end+1]
    z_m = z[idx_start:idx_end+1]
    t_m = t[idx_start:idx_end+1]
    speed_m = speed[idx_start:idx_end+1]

    # Remove duplicate timestamps
    _, unique_idx = np.unique(t_m, return_index=True)
    x_m = x_m[unique_idx]
    y_m = y_m[unique_idx]
    z_m = z_m[unique_idx]
    t_m = t_m[unique_idx]
    speed_m = speed_m[unique_idx]

    # -----------------------------
    # Acceleration
    # -----------------------------
    dt = np.diff(t_m)
    dv = np.diff(speed_m)
    acceleration = np.concatenate(([0], np.divide(dv, dt, out=np.zeros_like(dv), where=dt!=0)))

    if len(acceleration) >= WINDOW_LENGTH:
        acceleration_filtered = savgol_filter(acceleration, WINDOW_LENGTH, POLYORDER)
    else:
        acceleration_filtered = acceleration

    # -----------------------------
    # Path length and time
    # -----------------------------
    diffs = np.sqrt(np.diff(x_m)**2 + np.diff(y_m)**2 + np.diff(z_m)**2)
    path_length = np.sum(diffs)
    time_to_reach = t_m[-1] - t_m[0]

    # Store trajectory for inter-UAV distance calculation
    uav_groups[uav] = {"t": t_m, "x": x_m, "y": y_m, "z": z_m, "speed": speed_m, "acceleration": acceleration_filtered}

    # Store metrics for summary
    metrics_list[uav] = {
        "PathLength_m": path_length,
        "TimeToReach_s": time_to_reach,
        "AvgSpeed_mps": np.mean(speed_m),
        "MaxSpeed_mps": np.max(speed_m),
        "MaxAcceleration_mps2": np.max(acceleration_filtered)
    }

# -----------------------------
# Compute inter-UAV distance at each timestamp
# -----------------------------
all_times = np.unique(np.concatenate([data["t"] for data in uav_groups.values()]))
inter_uav_distances = {uav: [] for uav in uav_groups.keys()}

for t in all_times:
    positions = {}
    for uav, data in uav_groups.items():
        # Interpolate positions at this timestamp
        x_t = np.interp(t, data["t"], data["x"])
        y_t = np.interp(t, data["t"], data["y"])
        z_t = np.interp(t, data["t"], data["z"])
        positions[uav] = np.array([x_t, y_t, z_t])

    # Compute min distance to other UAVs
    for uav, pos in positions.items():
        min_dist = np.inf
        for other_uav, other_pos in positions.items():
            if uav == other_uav:
                continue
            dist = np.linalg.norm(pos - other_pos)
            if dist < min_dist:
                min_dist = dist
        inter_uav_distances[uav].append(min_dist)

# -----------------------------
# Add min distance to summary metrics
# -----------------------------
for uav, dists in inter_uav_distances.items():
    metrics_list[uav]["MinDistanceToOtherUAV_m"] = np.min(dists)

# -----------------------------
# Save results to CSV
# -----------------------------
with open(OUTPUT_FILE, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "UAV",
        "PathLength_m",
        "TimeToReach_s",
        "AvgSpeed_mps",
        "MaxSpeed_mps",
        "MaxAcceleration_mps2",
        "MinDistanceToOtherUAV_m"
    ])
    for uav, m in metrics_list.items():
        writer.writerow([
            uav,
            m["PathLength_m"],
            m["TimeToReach_s"],
            m["AvgSpeed_mps"],
            m["MaxSpeed_mps"],
            m["MaxAcceleration_mps2"],
            m["MinDistanceToOtherUAV_m"]
        ])

print(f"Metrics successfully saved to '{OUTPUT_FILE}'")


# -----------------------------
# Plot results
# -----------------------------
plt.figure(figsize=(15,10))

# Trajectory 3D
ax1 = plt.subplot(3,1,1, projection='3d')
for uav, data in uav_groups.items():
    ax1.plot(data["x"], data["y"], data["z"], label=uav)
ax1.set_xlabel("X [m]")
ax1.set_ylabel("Y [m]")
ax1.set_zlabel("Z [m]")
ax1.set_title("UAV Trajectories")
# ax1.legend()

# Set equal aspect for 3D axes
x_limits = ax1.get_xlim3d()
y_limits = ax1.get_ylim3d()
z_limits = ax1.get_zlim3d()
max_range = np.array([x_limits[1]-x_limits[0], y_limits[1]-y_limits[0], z_limits[1]-z_limits[0]]).max() / 2.0

mid_x = np.mean(x_limits)
mid_y = np.mean(y_limits)
mid_z = np.mean(z_limits)

ax1.set_xlim3d(mid_x - max_range, mid_x + max_range)
ax1.set_ylim3d(mid_y - max_range, mid_y + max_range)
ax1.set_zlim3d(mid_z - max_range, mid_z + max_range)

# Speed and Acceleration
ax2 = plt.subplot(3,1,2)
for uav, data in uav_groups.items():
    ax2.plot(data["t"], data["speed"], label=f"{uav} speed")
    ax2.plot(data["t"], data["acceleration"], '--', label=f"{uav} accel")
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Speed [m/s] / Acceleration [m/s²]")
ax2.set_title("Speed and Acceleration over Time")
ax2.grid(True)
# ax2.legend()

# Inter-UAV distance
ax3 = plt.subplot(3,1,3)
for uav, dist_list in inter_uav_distances.items():
    ax3.plot(all_times, dist_list, label=uav)
ax3.axhline(y=1.0, color='r', linestyle='--', label='Threshold 1 m')
ax3.set_xlabel("Time [s]")
ax3.set_ylabel("Distance to Closest UAV [m]")
ax3.set_title("Inter-UAV Distance over Time")
ax3.grid(True)
# ax3.legend()

plt.tight_layout()
plt.show()
