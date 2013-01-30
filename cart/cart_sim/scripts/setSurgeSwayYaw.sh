#!/bin/bash
rostopic pub -1 /pladypos/tauIn auv_msgs/BodyForceReq "{wrench: {force : {x: ${1},y: ${2},  z: 0.0}, torque: {x: 0.0,y: 0.0, z: ${3}}}}"
rostopic pub -1 /pladypos/modeIn std_msgs/Int32 1
