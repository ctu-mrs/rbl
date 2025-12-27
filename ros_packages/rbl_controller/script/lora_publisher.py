#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial
import time

# --- CONFIG ---
SERIAL_PORT = '/dev/ttyUSB0'   # FTDI device
BAUD_RATE = 115200
TOPIC_NAME = '/my_topic'
SEND_FREQ = 1.0  # Hz

# --- GLOBAL ---
last_message = ""

def callback(msg):
    global last_message
    last_message = msg.data

def main():
    global last_message

    # Initialize ROS node
    rospy.init_node('lora_bridge', anonymous=True)

    # Subscribe to topic
    rospy.Subscriber(TOPIC_NAME, String, callback)

    # Initialize serial
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        rospy.loginfo(f"Opened serial port {SERIAL_PORT} at {BAUD_RATE} bps")
    except serial.SerialException as e:
        rospy.logerr(f"Could not open serial port: {e}")
        return

    rate = rospy.Rate(SEND_FREQ)

    while not rospy.is_shutdown():
        if last_message:
            try:
                # Send message over LoRa
                ser.write((last_message + '\n').encode())
                rospy.loginfo(f"Sent: {last_message}")
            except serial.SerialException as e:
                rospy.logerr(f"Serial write error: {e}")
        rate.sleep()

    ser.close()

if __name__ == "__main__":
    main()
