#!/bin/bash
#Main definition of the machines
export ROBOT=real_pl

#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1

#Input sensors
export JOYSTICK=/dev/input/js1

#Env variables
export IS_PLADYPOS=1
export IS_SIM=0
export MODEL=`rospack find snippets`/data/models/pladypos_dvl.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal.yaml

#Frame env variables
export USE_TF_PREFIX=1
export TF_PREFIX=ldtravo

#Logging env variables
export LOGDIR=./
export ENABLE_LOGGING=1

#For SSH sessions
export ROSLAUNCH_SSH_UNKNOWN=1
