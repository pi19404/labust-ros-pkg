#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import NavSts;
from std_msgs.msg import Byte
import numpy
import math
      
class DynamicPositioning:
    def __init__(self):
        rospy.Subscriber("etaRef", NavSts, self.onRef)
        rospy.Subscriber("stateHat", NavSts, self.onStateHat);              
        self.stateHat = NavSts();
        self.out = rospy.Publisher("nuRef", BodyVelocityReq);
         
        w = numpy.array(rospy.get_param(
                            "dynamic_positioning/closed_loop_freq"));
        
        self.Kp = numpy.array([[2*w[0], 0],[0, 2*w[1]]],
                              dtype=numpy.float32);     
        self.Kp_yaw = 2*w[3];
        self.Kp_z = 2*w[2];
               
        self.state = numpy.array([0,0,0,0], numpy.float32);
        self.desired = numpy.array([0,0,0,0], numpy.float32);
        
        
        self.R = numpy.identity(2, numpy.float32);  
        
    def onRef(self,data):
        self.desired = numpy.array([data.position.north,
                                    data.position.east,
                                    data.position.depth,
                                    self.wrapAngle(data.orientation.yaw)], 
                                   dtype=numpy.float32);         
        print "Have ref:",self.desired                    
            
    def wrapAngle(self, angle):
        if angle>=math.pi:
            return -2*math.pi + angle;
        elif angle<-math.pi:
            return 2*math.pi + angle;
        else:
            return angle;
               
    def onStateHat(self,data):
        self.stateHat = data;
        self.state = numpy.array([data.position.north, 
                                  data.position.east, 
                                  data.position.depth, 
                                  self.wrapAngle(data.orientation.yaw)],
                                 dtype=numpy.float32);
        yaw = data.orientation.yaw;
        self.R = numpy.array([[math.cos(yaw),-math.sin(yaw)],
                              [math.sin(yaw),math.cos(yaw)]],numpy.float32);
                
    def step(self):
        d = self.desired - self.state;                                        
        pub = BodyVelocityReq();
        u = numpy.dot(numpy.transpose(self.R),numpy.dot(self.Kp,[d[0],d[1]]));
        pub.twist.linear.x = u[0];
        pub.twist.linear.y = u[1];
        pub.twist.linear.z = self.Kp_z*d[2];
        pub.twist.angular.z = self.Kp_yaw*self.wrapAngle(d[3]);
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub); 
        
if __name__ == "__main__":
    rospy.init_node("simpledp");
    dp = DynamicPositioning();
        
    rate = rospy.Rate(10.0);
        
    while not rospy.is_shutdown():
        dp.step();
        rate.sleep();
        
        
        