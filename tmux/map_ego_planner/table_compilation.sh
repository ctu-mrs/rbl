# echo 'length' && awk -F',' 'NR>1 {sum+=$2} END {print sum/(NR-1)}' summary_ego_2.csv
# echo 'time' && awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-1)}' summary_ego_2.csv
# echo 'vel' && awk -F',' 'NR>1 {sum+=$4} END {print sum/(NR-1)}' summary_ego_2.csv
# echo 'acc' && awk -F',' 'NR==2 {max=$6} NR>1 {if($6>max) max=$6} END {print max}' summary_ego_2.csv 
# echo 'min dist' && awk -F',' 'NR==2 {min=$7} NR>1 {if($7<min) min=$7} END {print min}' summary_ego_2.csv 


# echo 'length' && awk -F',' 'NR>1 {sum+=$2} END {print sum/(NR-1)}' summary_rbl_3.csv
# echo 'time' && awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-1)}' summary_rbl_3.csv
# echo 'vel' && awk -F',' 'NR>1 {sum+=$4} END {print sum/(NR-1)}' summary_rbl_3.csv
# echo 'acc' && awk -F',' 'NR==2 {max=$6} NR>1 {if($6>max) max=$6} END {print max}' summary_rbl_3.csv 
# echo 'min dist' && awk -F',' 'NR==2 {min=$7} NR>1 {if($7<min) min=$7} END {print min}' summary_rbl_3.csv 

echo 'length' && awk -F',' 'NR>1 {sum+=$2} END {print sum/(NR-3)}' summary_2drbl_1.csv
echo 'time' && awk -F',' 'NR>1 {sum+=$3} END {print sum/(NR-3)}' summary_2drbl_1.csv
echo 'vel' && awk -F',' 'NR>1 {sum+=$4} END {print sum/(NR-3)}' summary_2drbl_1.csv
echo 'acc' && awk -F',' 'NR==2 {max=$6} NR>1 {if($6>max) max=$6} END {print max}' summary_2drbl_1.csv 
echo 'min dist' && awk -F',' 'NR==2 {min=$7} NR>1 {if($7<min) min=$7} END {print min}' summary_2drbl_1.csv 

