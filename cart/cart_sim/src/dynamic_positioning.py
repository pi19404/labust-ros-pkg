#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import NavSts;
import numpy
import math
      
class DynamicPositioning:
    def __init__(self):
        #rospy.Subscriber("desiredPosition", VehiclePose, self.onRef)
        rospy.Subscriber("stateHat", NavSts, self.onStateHat);
        rospy.Subscriber("trackedNav", NavSts, self.onTrackedNav);
               
        self.stateHat = NavSts();
        
        self.out = rospy.Publisher("nuRef", BodyVelocityReq);
               
        self.Kp = numpy.reshape(
                    numpy.array(rospy.get_param("dynamic_positioning/Kp"),
                                numpy.float32), [2,2]);                
        self.Ki = numpy.reshape(
                    numpy.array(rospy.get_param("dynamic_positioning/Ki"),
                                numpy.float32), [2,2]);
        self.Ts = rospy.get_param("dynamic_positioning/Ts");
        
        self.mode = rospy.get_param("dynamic_positioning/mode");
               
        self.internalState = self.state = numpy.array([0,0], numpy.float32);
        self.desired = self.ff = numpy.array([0,0], numpy.float32);
        
        self.R = numpy.identity(2, numpy.float32);
        
    def onStateHat(self,data):
        self.stateHat = data;
        self.state = numpy.array([data.position.north, data.position.east],
                                 dtype=numpy.float32);
        yaw = data.orientation.yaw;
        self.R = numpy.array([[math.cos(yaw),math.sin(yaw)],
                              [-math.sin(yaw),math.cos(yaw)]],numpy.float32);
        
    def onRef(self,data):
        self.desired = numpy.array([data.position.north,data.position.east], 
                                   dtype=numpy.float32); 
    
    def onTrackedNav(self,data):
        self.onRef(data);
        self.ff[0] = data.body_velocity.x*math.cos(data.orientation.yaw);
        self.ff[1] = data.body_velocity.x*math.sin(data.orientation.yaw);
        
    def stepSS(self):
        d = self.state - self.desired;
        u = numpy.dot(numpy.transpose(self.R),(-numpy.dot(self.Kp,d) -
                                     self.internalState) + self.ff);
                                                                          
        '''Propagate integration after the output calculation'''
        self.internalState += numpy.dot(self.Ki,d)*self.Ts;
               
        pub = BodyVelocityReq();
        pub.twist.linear.x = u[0];
        pub.twist.linear.y = u[1];
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.yaw = pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub);
        
    def stepSH(self):
        d = self.desired - self.state;
        
        yaw_error = (math.atan2(d[1], d[0]) - self.stateHat.orientation.yaw)%(2*math.pi);
        
        if yaw_error > math.pi:
            yaw_error += -2*math.pi;
        elif yaw_error <= -math.pi:
            yaw_error += 2*math.pi;
                         
        pub = BodyVelocityReq();
        pub.twist.linear.x = self.Kp[0][0]*math.sqrt(d[0]*d[0] + d[1]*d[1]);
        pub.twist.angular.z = self.Kp[1][1]*yaw_error;
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.y = pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub);
        
    def step(self):
        if self.mode == 0:
            self.stepSS();
        else:
            self.stepSH();
         
        
if __name__ == "__main__":
    rospy.init_node("dpcontrol");
    dp = DynamicPositioning();
        
    rate = rospy.Rate(10.0);
        
    while not rospy.is_shutdown():
        dp.step();
        rate.sleep();
        
        
        