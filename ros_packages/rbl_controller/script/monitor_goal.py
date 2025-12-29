#!/usr/bin/env python3
import rospy
import yaml
import math
from nav_msgs.msg import Odometry

TOL = 0.3  # meters

class GoalMonitor:
    def __init__(self):
        cfg = rospy.get_param("~goals_file")
        with open(cfg, 'r') as f:
            self.goals = yaml.safe_load(f)["final_positions"]

        self.reached = {uav: False for uav in self.goals}

        for uav in self.goals:
            topic = f"/{uav}/estimation_manager/odom_main"
            rospy.Subscriber(topic, Odometry, self.cb, uav)

    def cb(self, msg, uav):
        g = self.goals[uav]
        p = msg.pose.pose.position
        d = math.sqrt((p.x-g["x"])**2 + (p.y-g["y"])**2 + (p.z-g["z"])**2)

        if d < TOL:
            self.reached[uav] = True

        if all(self.reached.values()):
            rospy.loginfo("✅ All UAVs reached goals")
            rospy.signal_shutdown("done")

if __name__ == "__main__":
    rospy.init_node("goal_monitor")
    GoalMonitor()
    rospy.spin()

# import rospy
# import yaml
# import math
# from nav_msgs.msg import Odometry
# from itertools import combinations

# TOL = 0.3  # meters

# class GoalMonitor:
#     def __init__(self):
#         cfg = rospy.get_param("~goals_file")
#         rospy.loginfo(f"Loading goals from {cfg}")

#         with open(cfg, "r") as f:
#             data = yaml.safe_load(f)

#         if "final_positions" not in data:
#             rospy.logerr("YAML file does not contain 'final_positions'")
#             rospy.signal_shutdown("invalid goals file")
#             return

#         self.goals = data["final_positions"]
#         self.reached = {uav: False for uav in self.goals}
#         self.positions = {}  # store latest positions
#         self.shutdown_called = False
#         self.min_distance = float("inf")  # initialize minimum distance

#         for uav in self.goals:
#             topic = f"/{uav}/estimation_manager/odom_main"
#             rospy.loginfo(f"Subscribing to {topic}")
#             rospy.Subscriber(topic, Odometry, self.cb, uav)

#     def cb(self, msg, uav):
#         p = msg.pose.pose.position
#         self.positions[uav] = (p.x, p.y, p.z)

#         # --- Track minimum inter-UAV distance ---
#         if len(self.positions) > 1:
#             for (u1, pos1), (u2, pos2) in combinations(self.positions.items(), 2):
#                 d = math.sqrt(
#                     (pos1[0]-pos2[0])**2 +
#                     (pos1[1]-pos2[1])**2 +
#                     (pos1[2]-pos2[2])**2
#                 )
#                 if d < self.min_distance:
#                     self.min_distance = d

#         # --- Goal check ---
#         if self.reached[uav]:
#             return

#         g = self.goals[uav]
#         d_goal = math.sqrt(
#             (p.x - g["x"]) ** 2 +
#             (p.y - g["y"]) ** 2 +
#             (p.z - g["z"]) ** 2
#         )

#         if d_goal < TOL:
#             self.reached[uav] = True
#             rospy.loginfo(f"✅ {uav} reached goal (error {d_goal:.2f} m)")

#         # --- All goals reached ---
#         if not self.shutdown_called and all(self.reached.values()):
#             self.shutdown_called = True
#             rospy.loginfo("🎯 All UAVs reached their goals")
#             rospy.loginfo(f"📏 Minimum inter-UAV distance during simulation: {self.min_distance:.2f} m")
#             rospy.signal_shutdown("done")

# if __name__ == "__main__":
#     rospy.init_node("goal_monitor")
#     GoalMonitor()
#     rospy.spin()
