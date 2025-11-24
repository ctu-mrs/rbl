#!/bin/bash
rostopic echo /map_generator/global_cloud -n 1 > map_2drbl_$1.txt && rosbag record -O 2drbl_odom_$1.bag /uav1/estimation_manager/odom_main



