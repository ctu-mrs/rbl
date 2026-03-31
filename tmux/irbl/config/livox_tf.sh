#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0.10 \
  --roll 0.0 \
  --pitch 0.35 \
  --yaw 0.0 \
  --frame-id $UAV_NAME/fcu \
  --child-frame-id $UAV_NAME/livox
