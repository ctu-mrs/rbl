#!/bin/bash
rostopic echo -b ego_odom.bag -p -a > ego_odom.csv
