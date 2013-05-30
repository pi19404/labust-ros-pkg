#!/bin/bash
rostopic pub /topside/joy sensor_msgs/Joy "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
axes:
 [0.0,$1,0.0,0.0,0.0,0.0]
buttons:
 [0,0]" -r 10

