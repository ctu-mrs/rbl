#!/usr/bin/env python3

import rospy
import serial
import json
import time
from nav_msgs.msg import Odometry

# ---------------- CONFIG ----------------
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TX_RATE_HZ = 1.0           # Send rate
USE_ROS = True             # False = fake data
# ----------------------------------------

latest_odom = None
rx_buffer = ""
seq_id = 0
last_rx_id = -1

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
        # Fake data mode
        payload = {
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
    global rx_buffer, last_rx_id

    data = ser.read(128).decode(errors="ignore")
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

            print("RX OK:", pkt)

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

    last_tx = time.time()
    tx_period = 1.0 / TX_RATE_HZ

    print("LoRa robust TX/RX started")

    try:
        while not (USE_ROS and rospy.is_shutdown()):
            now = time.time()

            # ---- TRANSMIT ----
            if now - last_tx >= tx_period:
                packet = build_packet()
                ser.write(packet.encode())
                ser.flush()
                # print("TX:", packet.strip())
                last_tx = now

            # ---- RECEIVE ----
            process_rx(ser)

            time.sleep(0.01)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
