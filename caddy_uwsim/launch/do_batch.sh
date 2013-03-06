#!/bin/bash
cpp -E -P -DPP_INNER_LOOP_W=${1} -DPP_OUTER_LOOP_W=${2} simulation_batch.yaml.meta > simulation_batch.yaml
cpp -E -P -D PP_LOG_FILE="\"/home/dnad/Development/Matlab/ros/pladypos_log_ideal_${1}_${2}.csv\"" simulation_batch.launch.meta > simulation_batch.launch
