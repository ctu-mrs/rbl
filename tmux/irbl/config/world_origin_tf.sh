#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 \
  --y 0 \
  --z 0.0 \
  --roll 0.0 \
  --pitch 0.0 \
  --yaw 0.0 \
  --frame-id $UAV_NAME/point_lio_origin \
  --child-frame-id $UAV_NAME/world_origin
