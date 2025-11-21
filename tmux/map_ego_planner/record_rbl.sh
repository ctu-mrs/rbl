#!/bin/bash
rostopic echo /map_generator/global_cloud -n 1 > map_rbl.txt && rosbag record -O rbl_odom.bag /uav1/estimation_manager/odom_main 



