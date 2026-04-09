#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from mavros_msgs.msg import State, RCIn
from mrs_msgs.srv import Vec4, Float64Srv, String
from nav_msgs.msg import Odometry


class RCGoalController(Node):
    def __init__(self):
        super().__init__('rc_goal_controller')

        # -----------------------
        # State
        # -----------------------
        self.current_mode = ""
        self.current_pose = None

        # Goal
        self.last_goal = [0.0, 0.0, 2.0, 0.0]

        # betaD
        self.betaD = 0.0
        self.last_betaD_sent = None
        self.beta_threshold = 0.1

        # Params
        self.step_size = 0.4
        self.deadzone = 50

        # Estimator switching
        self.last_ch9_state = "NONE"
        self.switch_time = None
        self.waiting_for_fresh_pose = False

        self.ESTIMATOR_MAP = {
            "MID": "point_lio",
            "HIGH": "gps_garmin",
        }

        # -----------------------
        # Subscribers
        # -----------------------
        self.create_subscription(State, '/uav1/mavros/state', self.state_cb, 10)
        self.create_subscription(RCIn, '/uav1/mavros/rc/in', self.rc_cb, 10)
        self.create_subscription(Odometry, '/uav1/estimation_manager/odom_main', self.odom_cb, 10)

        # -----------------------
        # Service clients
        # -----------------------
        self.goal_cli = self.create_client(Vec4, '/uav1/rbl_controller/goto')
        self.beta_cli = self.create_client(Float64Srv, '/uav1/rbl_controller/set_betaD')
        self.estimator_cli = self.create_client(String, '/uav1/estimation_manager/change_estimator')

        self.wait_for_services()

        self.get_logger().info('RC Goal Controller ready (event-driven)')

    # -----------------------
    # Setup
    # -----------------------
    def wait_for_services(self):
        while not self.goal_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /goto service...')
        while not self.beta_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_betaD service...')
        while not self.estimator_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for estimator service...')

    # -----------------------
    # Callbacks
    # -----------------------
    def odom_cb(self, msg):
        self.current_pose = msg

        # 🔥 Handle estimator switch HERE (event-driven)
        if self.waiting_for_fresh_pose:
            self.handle_post_switch_hold(msg)

    def state_cb(self, msg):
        self.current_mode = msg.mode

    def rc_cb(self, msg):
        if len(msg.channels) < 9:
            return

        ch3 = msg.channels[0]
        ch2 = msg.channels[1]
        ch1 = msg.channels[2]
        ch9 = msg.channels[8]

        if ch9 < 1400:
            return

        current_switch_state = self.get_switch_state(ch9)

        # -----------------------
        # Estimator switching
        # -----------------------
        if current_switch_state != self.last_ch9_state:
            if current_switch_state in self.ESTIMATOR_MAP:
                self.start_estimator_switch(current_switch_state)

        self.last_ch9_state = current_switch_state

        # -----------------------
        # betaD
        # -----------------------
        self.handle_beta(ch2)

        # -----------------------
        # Position control
        # -----------------------
        goal_changed = self.handle_position(ch1, ch3)

        if goal_changed:
            self.get_logger().info(f"Sending goal: {self.last_goal}")
            self.send_goal()

    # -----------------------
    # Estimator switching
    # -----------------------
    def get_switch_state(self, ch9):
        if 1400 < ch9 < 1800:
            return "MID"
        elif ch9 > 1900:
            return "HIGH"
        return "NONE"

    def start_estimator_switch(self, state):
        estimator = self.ESTIMATOR_MAP[state]

        self.send_estimator(estimator)

        # Mark switch moment
        self.switch_time = self.get_clock().now()

        # Wait for fresh odometry
        self.waiting_for_fresh_pose = True

        self.get_logger().info(f"Waiting for fresh pose from {estimator}...")

    def handle_post_switch_hold(self, msg):
        pose_time = Time.from_msg(msg.header.stamp)

        # Reject old data
        if pose_time.nanoseconds <= self.switch_time.nanoseconds:
            return

        pos = msg.pose.pose.position
        self.last_goal = [pos.x, pos.y, pos.z, 0.0]

        self.get_logger().info(f"[HOLD AFTER SWITCH] {self.last_goal}")
        self.send_goal()

        # Done
        self.waiting_for_fresh_pose = False

    # -----------------------
    # betaD
    # -----------------------
    def handle_beta(self, ch2):
        new_beta = self.map_range(ch2)

        if self.last_betaD_sent is None or abs(new_beta - self.last_betaD_sent) > self.beta_threshold:
            self.betaD = new_beta
            self.last_betaD_sent = new_beta

            self.get_logger().info(f"betaD → {self.betaD:.2f}")
            self.send_beta()

    # -----------------------
    # Position control
    # -----------------------
    def handle_position(self, ch1, ch3):
        dx = self.process_channel(ch1)
        dy = -self.process_channel(ch3)

        if dx == 0.0 and dy == 0.0:
            return False

        self.last_goal[0] += dx * self.step_size
        self.last_goal[1] += dy * self.step_size

        return True

    # -----------------------
    # Helpers
    # -----------------------
    def process_channel(self, pwm):
        if abs(pwm - 1500) < self.deadzone:
            return 0.0
        return (pwm - 1500.0) / 500.0

    def map_range(self, pwm, in_min=983.0, in_max=2006.0, out_min=0.1, out_max=20.0):
        pwm = max(min(pwm, in_max), in_min)
        return out_min + (pwm - in_min) * (out_max - out_min) / (in_max - in_min)

    # -----------------------
    # Service calls
    # -----------------------
    def send_goal(self):
        req = Vec4.Request()
        req.goal = self.last_goal
        future = self.goal_cli.call_async(req)
        future.add_done_callback(self.goal_response_cb)

    def send_beta(self):
        req = Float64Srv.Request()
        req.value = float(self.betaD)
        self.beta_cli.call_async(req)

    def send_estimator(self, name):
        req = String.Request()
        req.value = name
        self.estimator_cli.call_async(req)
        self.get_logger().info(f"Switched estimator → {name}")

    def goal_response_cb(self, future):
        try:
            response = future.result()
            if hasattr(response, 'success') and response.success:
                self.get_logger().info("Goal sent successfully")
            else:
                self.get_logger().warn("Goal rejected")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


# -----------------------
# Main
# -----------------------
def main(args=None):
    rclpy.init(args=args)
    node = RCGoalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
