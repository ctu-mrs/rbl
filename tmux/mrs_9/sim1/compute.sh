#!/bin/bash

awk -F',' 'NR>1 {sum+=$2} END {print sum/(NR-1)}' metric
awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-1)}' metric
awk -F',' 'NR>1 {sum+=$4} END {print sum/(NR-1)}' metric
