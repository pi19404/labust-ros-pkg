#!/bin/bash
export ROBOT=sim
export LOCATION=jarun
export JOYSTICK=/dev/input/js1
export PACKAGE=cart2
export USE_UWSIM=0
export USE_RVIZ=1
export IS_SIM=0
export IS_REMOTE=0
export USE_BENCH_RADIO=1
export NO_NOISE=1
export LOGDIR=`pwd`/logs/
mkdir -p ${LOGDIR}
export ENABLE_LOGGING=1
export USE_VR=0
export USE_CART=0
export SIM_MODEL=`rospack find cart2`/data/config/cart2_model.xml
export YAML_MODEL=`rospack find cart2`/data/config/model.yaml
