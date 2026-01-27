#!/bin/bash
seq 1 30 | xargs -n1 -I{} python3 data.py --sim {}
cd sim2d_02 
rm uav_metrics_summary_ok.csv
cp uav_metrics_summary1.csv uav_metrics_summary_ok.csv
awk 'FNR>1' uav_metrics_summary{2..30}.csv >> uav_metrics_summary_ok.csv
awk -F',' 'NR==2 {min=$7} NR>2 && $7<min {min=$7} END{print min}' uav_metrics_summary_ok.csv

awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-1)}' uav_metrics_summary_ok.csv
