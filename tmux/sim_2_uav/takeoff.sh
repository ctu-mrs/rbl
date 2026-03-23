#!/bin/bash

UAV_NAME=${UAV_NAME:-uav1}

echo "arming ${UAV_NAME}"

ros2 service call /${UAV_NAME}/hw_api/arming std_srvs/srv/SetBool '{"data": true}'

sleep 1.0

echo "toggling offboard ${UAV_NAME}"

ros2 service call /${UAV_NAME}/hw_api/offboard std_srvs/srv/Trigger '{}'
