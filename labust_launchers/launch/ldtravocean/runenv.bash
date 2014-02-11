#!/bin/bash
#Main definition of the machines
export ROBOT=sim

#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1

#Input sensors
export JOYSTICK=/dev/input/js1

#Simulation env variables
export IS_SIM=0
export USE_NOISE=0
export MODEL=`rospack find snippets`/data/models/ldtrav_rov.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal_vertical.yaml
export USE_DIRECT=0
export USE_EXTENDED_NAV=1

#Frame env variables
export USE_TF_PREFIX=1
export TF_PREFIX=ldtravo

#Logging env variables
export LOGDIR=./
export ENABLE_LOGGING=1
