#!/bin/bash
#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1

#Input
export JOYSTICK=/dev/input/js1

#Simulation variables
export IS_SIM=1
export SIM_DIVER=1

#Vehicle variables
export MODEL=`rospack find snippets`/data/models/pladypos.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal.yaml

#Frame description
export USE_TF_PREFIX=1
export TF_PREFIX=pladypos

#USBL variables
export USE_USBL_MANAGER=0
export USE_USBL=0

#Logging env variables
export ENABLE_LOGGING=1
export LOGDIR=./

#Control configuration
export USE_IDENTIFICATION=1
export USE_MULTIMASTER=1
