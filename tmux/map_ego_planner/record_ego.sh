#!/bin/bash


rostopic echo /map_generator/global_cloud -n 1 > map_ego_$1.txt && rosbag record -O ego_odom_$1.bag /drone_0_odom_visualization/velocity

