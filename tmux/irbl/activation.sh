#!/bin/bash
ros2 service call /uav2/rbl_controller/goto mrs_msgs/srv/Vec4 "{goal: [45.0, 0.0, 2.0, 0.0]}"

ros2 service call /uav2/rbl_controller/activation std_srvs/srv/Trigger
