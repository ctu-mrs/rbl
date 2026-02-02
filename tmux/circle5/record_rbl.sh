#!/bin/bash
rosparam set /use_sim_time true
rosbag record -O rbl_odom_$1.bag \
  /uav1/estimation_manager/odom_main \
  /uav2/estimation_manager/odom_main \
  /uav3/estimation_manager/odom_main \
  /uav4/estimation_manager/odom_main \
  /uav5/estimation_manager/odom_main \
  /uav6/estimation_manager/odom_main \
  /uav7/estimation_manager/odom_main \
  /uav8/estimation_manager/odom_main \
  /uav9/estimation_manager/odom_main \
  /uav10/estimation_manager/odom_main \
  /tf \
  /tf_static \
  /clock
