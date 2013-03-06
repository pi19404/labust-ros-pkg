#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from auv_msgs.msg import NavSts;
import numpy
import math
import sys

def wrapAngle(angle):
    if angle>math.pi:
        return angle - 2* math.pi;
    elif angle<-math.pi:
        return 2*math.pi + angle;
    else:
        return angle;
        
if __name__ == "__main__":
    rospy.  init_node("refgen");
    out = rospy.Publisher("etaRef", NavSts);
        
    rate = rospy.Rate(1);
        
    ref = NavSts();
    while not rospy.is_shutdown():
        refs = [float(x) for x  in str(sys.stdin.readline()).split()];
        ref.position.north = refs[0];
        ref.position.east = refs[1];
        ref.position.depth = refs[2];
        ref.orientation.yaw = wrapAngle(refs[3]);
            
        out.publish(ref);
        rate.sleep();
        
        
        