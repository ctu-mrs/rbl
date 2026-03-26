#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, RCIn
from mrs_msgs.srv import Vec4
from nav_msgs.msg import Odometry

class RCGoalController(Node):
    def __init__(self):
        super().__init__('rc_goal_controller')
        # Current mode and last goal
        self.current_mode = ""
        self.last_goal = [0.0, 0.0, 2.0, 0.0]  # x, y, z, yaw

        self.current_pose = None
        # Parameters
        self.step_size = 0.2
        self.deadzone = 50  # ignore small stick movements

        # Subscribers
        self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.create_subscription(RCIn, '/uav1/mavros/rc/in', self.rc_cb, 10)

        self.create_subscription(
            Odometry,
            '/uav1/estimation_manager/odom_main',
            self.odom_cb,
            10
        )
        # Service client
        self.cli = self.create_client(Vec4, '/uav1/rbl_controller/goto')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /uav1/rbl_controller/goto service...')

        self.get_logger().info('RC Goal Controller ready')

    # -----------------------
    # Callbacks
    # -----------------------
    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
    def state_cb(self, msg):
        self.current_mode = msg.mode

    def rc_cb(self, msg):
        if msg.channels[8] < 1400:
            return
        if len(msg.channels) < 3:
            return

        # CH1 = left/right (x), CH3 = forward/back (y)
        ch3 = msg.channels[0]
        ch1 = msg.channels[2]
        ch9 = msg.channels[8]

        dx = self.process_channel(ch1)
        dy = -self.process_channel(ch3)

        if dx == 0.0 and dy == 0.0:
            return



        self.last_goal[0] += dx * self.step_size
        self.last_goal[1] += dy * self.step_size

        if ch9 > 1550:
            if self.current_pose is None:
                self.get_logger().warn("No odometry yet")
                return

            x = self.current_pose.position.x
            y = self.current_pose.position.y
            z = self.current_pose.position.z

            self.last_goal = [x, y, z, 0.0]

            self.get_logger().info(f"[HOLD] Setting goal to current position: {self.last_goal}")
            # self.send_goal()

        self.get_logger().info(f"New goal: {self.last_goal}")
        self.send_goal()

    # -----------------------
    # Helpers
    # -----------------------
    def process_channel(self, pwm):
        if abs(pwm - 1500) < self.deadzone:
            return 0.0
        return (pwm - 1500.0) / 500.0  # normalize to [-1,1]

    def send_goal(self):
        req = Vec4.Request()
        req.goal = self.last_goal
        future = self.cli.call_async(req)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info("Goal sent successfully")
            else:
                self.get_logger().warn("Goal rejected")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = RCGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
