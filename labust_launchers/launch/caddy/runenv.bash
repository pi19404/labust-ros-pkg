#!/bin/bash
export ROBOT=real
export JOYSTICK=/dev/input/js1
export LOCATION=colentum
export IS_SIM=0
export USE_USBL=0
export USE_USBL_MANAGER=0
export MODEL=`rospack find snippets`/data/models/pladypos.yaml
export USE_TF_PREFIX=1
export TF_PREFIX=pladypos
export USE_LOCAL_FIX=1
export ENABLE_LOGGING=1
export LOGDIR=~/logs
export ROSLAUNCH_SSH_UNKNOWN=1
