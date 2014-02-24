#!/bin/bash
#Main definition of the machines
export ROBOT=sim

#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1

#Input sensors
export JOYSTICK=/dev/input/js0

#Simulation env variables
export IS_SIM=1
export USE_NOISE=1
#export MODEL=`rospack find snippets`/data/models/ldtrav_rov.yaml
export MODEL=`rospack find snippets`/data/models/ldtrav_trench.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal_vertical.yaml
export USE_DIRECT=0
export USE_EXTENDED_NAV=1
export USE_NZ_NAV=0

#Frame env variables
export USE_TF_PREFIX=1
export TF_PREFIX=ldtravo

#Logging env variables
export ENABLE_LOGGING=1
export LOGDIR=./
