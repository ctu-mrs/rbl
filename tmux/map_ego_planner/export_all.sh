#!/bin/bash

# n=5
# cd sim_1 
# for ((i=1; i<=n; i++)); do
#     ./export_odom.sh ego_odom_${i}.bag &
#     ./export_odom.sh rbl_odom_${i}.bag &
#     ./export_odom.sh 2drbl_odom_${i}.bag &
#     cd ../sim_2 & ./export_odom.sh ego_odom_${i}.bag &
#     ./export_odom.sh rbl_odom_${i}.bag &
#     ./export_odom.sh 2drbl_odom_${i}.bag &
#     cd ../sim_3 & ./export_odom.sh ego_odom_${i}.bag &
#     ./export_odom.sh rbl_odom_${i}.bag &
#     ./export_odom.sh 2drbl_odom_${i}.bag &
# done

source ~/open3d_venv/bin/activate
# python3 data_ego_map.py --sim 1
# python3 data_ego_map.py --sim 2
# python3 data_ego_map.py --sim 3
python3 data_ego_map.py --sim 4

# python3 data_rbl_map.py --sim 1
# python3 data_rbl_map.py --sim 2
# python3 data_rbl_map.py --sim 3

# python3 data_2drbl_map.py --sim 1
# python3 data_2drbl_map.py --sim 2
# python3 data_2drbl_map.py --sim 3



