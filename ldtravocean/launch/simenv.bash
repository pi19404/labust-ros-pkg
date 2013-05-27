#!/bin/bash
export ROBOT=sim
export LOCATION=jarun
export JOYSTICK=/dev/input/js0
export PACKAGE=ldtravocean
export USE_UWSIM=1
export USE_RVIZ=1
export IS_SIM=1
export IS_REMOTE=0
export NO_NOISE=1
export LOGDIR=`pwd`/logs/
mkdir -p ${LOGDIR}
export ENABLE_LOGGING=0
export USE_VR=0
export USE_CART=0
export SIM_MODEL=`rospack find ldtravocean`/data/config/model.xml
export YAML_MODEL=`rospack find ldtravocean`/data/config/model.yaml
