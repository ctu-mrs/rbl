
#!/usr/bin/env python3

import rospy
import serial
import json
import time
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Point

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TX_RATE_HZ = 1.0            # Nominal send rate (Hz)
USE_ROS = True               # False = fake data
NODE_ID = "uav1"             # Unique node ID
NODE_SLOT = 0                # 0-based slot index
NUM_NODES = 5                # Total number of nodes
SLOT_JITTER = 0.1 # Random jitter within slot (seconds)
RX_BUFFER_SIZE = 1024        # Serial read buffer size
# ----------------------------------------

latest_odom = None
rx_buffer = ""
seq_id = 0
last_rx_id = -1

# Dictionary to cache publishers per frame_id
publishers = {}

# ---------- ROS CALLBACK ----------
def odom_callback(msg):
    global latest_odom
    latest_odom = msg

# ---------- PACKET BUILD ----------
def build_packet():
    global seq_id

    if USE_ROS and latest_odom:
        msg = latest_odom
        payload = {
            "node_id": NODE_ID,
            "frame_id": msg.header.frame_id,
            "id": seq_id,
            "t": round(msg.header.stamp.to_sec(), 2),
            "x": round(msg.pose.pose.position.x, 2),
            "y": round(msg.pose.pose.position.y, 2),
            "z": round(msg.pose.pose.position.z, 2),
            "qx": round(msg.pose.pose.orientation.x, 3),
            "qy": round(msg.pose.pose.orientation.y, 3),
            "qz": round(msg.pose.pose.orientation.z, 3),
            "qw": round(msg.pose.pose.orientation.w, 3),
        }
    else:
        payload = {
            "node_id": NODE_ID,
            "frame_id": "odom",
            "id": seq_id,
            "t": round(time.time(), 2),
            "x": round(seq_id * 0.1, 2),
            "y": 0.0,
            "z": 3.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
        }

    seq_id += 1
    return f"<{json.dumps(payload, separators=(',', ':'))}>\n"

# ---------- RECEIVE HANDLER ----------
def process_rx(ser):
    global rx_buffer, last_rx_id, publishers

    data = ser.read(RX_BUFFER_SIZE).decode(errors="ignore")
    if not data:
        return

    rx_buffer += data

    while "<" in rx_buffer and ">" in rx_buffer:
        start = rx_buffer.find("<")
        end = rx_buffer.find(">", start)
        raw = rx_buffer[start + 1:end]
        rx_buffer = rx_buffer[end + 1:]

        try:
            pkt = json.loads(raw)
            pkt_id = pkt.get("id", None)

            if pkt_id is not None:
                if pkt_id == last_rx_id:
                    print("RX duplicate:", pkt_id)
                elif pkt_id != last_rx_id + 1 and last_rx_id != -1:
                    print("RX packet jump:", last_rx_id, "→", pkt_id)
                last_rx_id = pkt_id

            frame_id = pkt.get("frame_id", "odom")
            topic_name = f"{frame_id}/lora/odom"

            if frame_id not in publishers:
                publishers[frame_id] = rospy.Publisher(topic_name, Odometry, queue_size=10)
                rospy.loginfo(f"Created publisher for topic: {topic_name}")

            pub = publishers[frame_id]

            odom_msg = Odometry()
            odom_msg.header.stamp = rospy.Time.now()
            odom_msg.header.frame_id = frame_id
            odom_msg.child_frame_id = pkt.get("node_id", "base_link")

            odom_msg.pose.pose.position = Point(pkt.get("x", 0.0),
                                                pkt.get("y", 0.0),
                                                pkt.get("z", 0.0))
            odom_msg.pose.pose.orientation = Quaternion(pkt.get("qx", 0.0),
                                                        pkt.get("qy", 0.0),
                                                        pkt.get("qz", 0.0),
                                                        pkt.get("qw", 1.0))
            pub.publish(odom_msg)

            print(f"RX OK from {pkt.get('node_id', 'unknown')}: {pkt}")

        except json.JSONDecodeError:
            print("RX bad JSON:", raw)

# ---------- MAIN ----------
def main():
    global seq_id

    if USE_ROS:
        rospy.init_node("lora_robust_tx_rx", anonymous=True)
        rospy.Subscriber(
            "/uav1/estimation_manager/odom_main",
            Odometry,
            odom_callback,
            queue_size=1
        )

    ser = serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        timeout=0,
        write_timeout=0
    )

    # ----- Compute time slot -----
    tx_period_base = 1.0 / TX_RATE_HZ
    slot_duration = tx_period_base / NUM_NODES
    initial_offset = NODE_SLOT * slot_duration
    print(f"Node {NODE_ID} starting in slot {NODE_SLOT}, initial offset {initial_offset:.3f}s")
    time.sleep(initial_offset)

    last_tx = time.time()

    print("LoRa robust TX/RX started")

    try:
        while not (USE_ROS and rospy.is_shutdown()):
            now = time.time()
            # Slot-based transmit with small jitter
            tx_period = tx_period_base / NUM_NODES + random.uniform(-SLOT_JITTER, SLOT_JITTER)

            if now - last_tx >= tx_period:
                packet = build_packet()
                ser.write(packet.encode())
                ser.flush()
                last_tx = now

            process_rx(ser)
            time.sleep(0.01)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
