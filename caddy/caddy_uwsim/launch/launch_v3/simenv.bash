#!/bin/bash
export ROBOT=sim
export LOCATION=labos
export JOYSTICK=/dev/input/js1
export IS_SIM=1
export SIM_DIVER=1
export MODEL=`rospack find snippets`/data/models/pladypos.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/x_horizontal.yaml
export USE_TF_PREFIX=1
export TF_PREFIX=pladypos
export USE_LOCAL_FIX=1
export USE_USBL_MANAGER=0
export USE_USBL=0
export USE_UWSIM=1
export USE_LOCAL_FIX=1
export LOGDIR=./
