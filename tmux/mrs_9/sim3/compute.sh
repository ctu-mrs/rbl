#!/bin/bash
# tail -n +2 uav_metrics_summary1.csv > metric
# tail -n +2 uav_metrics_summary2.csv >> metric
# tail -n +2 uav_metrics_summary3.csv >> metric
# tail -n +2 uav_metrics_summary4.csv >> metric
# tail -n +2 uav_metrics_summary5.csv >> metric
# tail -n +2 uav_metrics_summary6.csv >> metric
# tail -n +2 uav_metrics_summary7.csv >> metric
# tail -n +2 uav_metrics_summary8.csv >> metric
# tail -n +2 uav_metrics_summary9.csv >> metric
# tail -n +2 uav_metrics_summary10.csv >> metric

awk -F',' 'NR>1 {sum+=$2} END {print sum/(NR-1)}' metric
awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-1)}' metric
awk -F',' 'NR>1 {sum+=$4} END {print sum/(NR-1)}' metric
