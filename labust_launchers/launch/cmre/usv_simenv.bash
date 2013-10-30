#!/bin/bash
export ROBOT=sim
export LOCATION=labos
export JOYSTICK=/dev/input/js1
export IS_SIM=1
export MODEL=`rospack find snippets`/data/models/cart2.yaml
export ALLOCATION_MODEL=`rospack find snippets`/data/allocations/differential_horizontal.yaml
export USE_TF_PREFIX=1
export TF_PREFIX=usv
export USE_LOCAL_FIX=1
export USE_UWSIM=1
export USE_LOCAL_FIX=1
export LOGDIR=./
