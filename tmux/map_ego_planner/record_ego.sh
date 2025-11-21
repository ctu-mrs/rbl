#!/bin/bash


rostopic echo /map_generator/global_cloud -n 1 > map_ego.txt && rosbag record -O ego_odom.bag /drone_0_odom_visualization/velocity && rostopic echo -b ego_odom.bag -p -a > ego_odom.csv

