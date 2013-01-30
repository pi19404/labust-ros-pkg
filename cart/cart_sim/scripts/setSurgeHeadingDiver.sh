#!/bin/bash
rostopic pub -1 /diver/refIn auv_msgs/VehiclePose "{position:{north: 0.0,east: 0.0,depth: ${3}},orientation:{roll: 0.0,pitch: 0,yaw: ${2}}}"
rostopic pub -1 /diver/tauIn auv_msgs/BodyForceReq "{wrench: {force : {x: ${1},y: 0.0,  z: 0.0}, torque: {x: 0.0,y: 0.0, z: 0.0}}}"
rostopic pub -1 /diver/modeIn std_msgs/Int32 2
