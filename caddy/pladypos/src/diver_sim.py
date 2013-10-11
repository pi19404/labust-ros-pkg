#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from sensor_msgs.msg import Joy;
from auv_msgs.msg import NavSts;

import numpy
import math
      
class DiverSim:
    def __init__(self):
        '''Setup subscribers'''
        rospy.Subscriber("joy", Joy, self.onJoy);     
        '''Setup publishers'''
        self.out = rospy.Publisher("TrackPoint", NavSts);
        self.point = NavSts();
        self.u = 1.0;
        self.theta = 0.3;
        self.Ts = 0.1;   
        self.point.global_position.latitude = 44;
        self.point.global_position.longitude = 15.305;
        self.point.position_variance.north = 167.123;
        self.point.position_variance.east = 155.623;
    
    def onJoy(self,joy):
        self.point.global_position.latitude += 0.0001*joy.axes[1]*self.Ts*self.u;
        self.point.global_position.longitude += 0.0001*joy.axes[0]*self.u*self.Ts;
        #self.point.position.north += joy.axes[1]*self.Ts*self.u*math.cos(self.point.orientation.yaw);
        self.point.position.north += joy.axes[1]*self.Ts*self.u;
        #self.point.position.east += joy.axes[1]*self.u*self.Ts*math.sin(self.point.orientation.yaw);
        self.point.position.east += joy.axes[0]*self.u*self.Ts;
        self.point.orientation.yaw += -joy.axes[2]*self.theta*self.Ts; 
        self.point.body_velocity.x = joy.axes[1]*self.u;
        self.out.publish(self.point);
        
if __name__ == "__main__":
    rospy.init_node("diver_sim");
    dp = DiverSim();    
    rospy.spin();
        
