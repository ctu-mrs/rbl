#!/usr/bin/env bash
START=22
END=50

for N in $(seq $START $END); do
  echo "Starting simulation $N"
  ./start.sh "$N"

  echo "Waiting for all UAVs to reach goals..."
# Wait until /goal_monitor node appears
  while ! rosnode list | grep -q /goal_monitor; do
    sleep 1
  done

  # Now wait until the node disappears (all UAVs reached goals)
  while rosnode list | grep -q /goal_monitor; do
    sleep 1
  done

  echo " All UAVs reached goals"
  sleep 2
  tmux -L mrs send-keys -t simulation:rosbag.0 C-c
  sleep 2
  tmux -L mrs kill-session -t simulation
  echo "Simulation $N finished"
  rostopic echo -b rbl_odom_$N.bag -p -a > rbl_odom_$N.bag.csv
  sleep 2
done



