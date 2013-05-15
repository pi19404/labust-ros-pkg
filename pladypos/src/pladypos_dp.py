#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import BodyForceReq;
from auv_msgs.msg import NavSts;
from dynamic_reconfigure.server import Server
from pladypos.cfg import DPSettingsConfig

import numpy
import math
      
class DynamicPositioning:
    def __init__(self):
        '''Setup subscribers'''
        rospy.Subscriber("stateHat", NavSts, self.onStateHat);
        rospy.Subscriber("tauAch", BodyForceReq, self.onTauAch);
        self.trackingSub = rospy.Subscriber("trackedNav", NavSts, self.onTrackedNav);
        
        '''Setup publishers'''
        self.out = rospy.Publisher("nuRef", BodyVelocityReq);
        
        '''Setup dynamic reconfigure'''
        self.srv = Server(DPSettingsConfig, self.dynrec)
 
        '''Setup gains'''
        w = numpy.array(rospy.get_param(
                            "dynamic_positioning/closed_loop_freq"));     
        self.Kp = numpy.array([[2*w[0], 0],[0, 2*w[1]]],
                              dtype=numpy.float32);
        self.Ki = numpy.array([[w[0]*w[0], 0], [0, w[1]*w[1]]], 
                               dtype=numpy.float32);                     
        self.Ts = rospy.get_param("dynamic_positioning/Ts");
        self.Kyaw = 2*w[2];
        self.Ki = w[2]*w[2];
               
        self.internalState = numpy.array([0,0], numpy.float32);
        self.state = numpy.array([0,0], numpy.float32);
        self.desired = numpy.array([0,0], numpy.float32);
        self.externalRef = numpy.array([0,0], numpy.float32); 
        self.ff = numpy.array([0,0], numpy.float32);
        self.windup = numpy.array([0,0,0], dtype=numpy.int8);
        self.lastW = numpy.array([0,0,0], dtype=numpy.int8);
        self.lastI = numpy.array([0,0], numpy.float32);
        self.lastYawI = 0;
        self.yawInternalState = 0;
        
        self.R = numpy.identity(2, numpy.float32);
        self.uk_1 = 0;
        self.lastTime = rospy.Time.now();
        
        '''Backward suff'''
        self.lastP = numpy.array([0,0]);
        self.lastFF = numpy.array([0,0]);
        
        '''Init variables'''
        self.stateHat = NavSts();
        self.yaw = 0;
    
    def dynrec(self,config, level):
        self.config = config;
        
        if level == 32 and len(config['TrackVariable']): 
            self.trackingSub.unregister();
            self.trackingSub = rospy.Subscriber(config['TrackVariable'],NavSts,self.onTrackedNav)
            
        return config   
    
    def onTauAch(self,tau):
        self.windup = numpy.array([tau.disable_axis.x, tau.disable_axis.y, tau.disable_axis.yaw])
                
    def onStateHat(self,data):
        self.stateHat = data;
        self.state = numpy.array([data.position.north, data.position.east],
                                 dtype=numpy.float32);
        self.yaw = data.orientation.yaw;
        self.R = numpy.array([[math.cos(self.yaw),-math.sin(self.yaw)],
                              [math.sin(self.yaw),math.cos(self.yaw)]],numpy.float32);
                              
        self.step();
        
    def onTrackedNav(self,data):
        self.externalRef = numpy.array([data.position.north,data.position.east], 
                                   dtype=numpy.float32); 
        self.ff[0] = data.body_velocity.x*math.cos(data.orientation.yaw);
        self.ff[1] = data.body_velocity.x*math.sin(data.orientation.yaw);                       
    
    def stepSSbackward(self):              
        if self.config['Manual']:
            self.desired = numpy.array([
                        self.config['Northing'],
                        self.config['Easting']],dtype=numpy.float32);
        else:
            self.desired = self.externalRef;
            
        d = self.desired - self.state;
       
        #\todo Use here the PIDController written in C instead of a separate type controller
        self.internalState += numpy.dot(self.Kp,d) - self.lastP;
        self.lastP = numpy.dot(self.Kp,d);
        
        self.internalState += numpy.dot(self.R,self.ff) - self.lastFF;
        self.lastFF = numpy.dot(self.R,self.ff);
        
        '''Handle windup'''                   
        if (numpy.linalg.norm(self.lastW, 2) == 0) and (numpy.linalg.norm(self.windup, 2) != 0):
            self.internalState -= self.lastI;
            self.lastI = 0;
        
        if numpy.linalg.norm(self.windup, 2) == 0:
            self.internalState += numpy.dot(self.Ki,d)*self.Ts;          
            self.lastI += numpy.dot(self.Ki,d)*self.Ts;
        
        self.lastW = 1*self.windup;
        
        u = numpy.dot(numpy.transpose(self.R),self.internalState);
            
        pub = BodyVelocityReq();
        pub.twist.linear.x = u[0];
        pub.twist.linear.y = u[1];
        error = self.config['Heading'] - self.yaw;
        
        if error > numpy.pi:
            error = error - 2.0*numpy.pi;
        if error < -numpy.pi:
            error = 2.0*numpy.pi + error;
             
        if not self.windup[2]:
            self.yawInternalState += self.Ki*error*self.Ts;
        pub.twist.angular.z = self.Kyaw*error + self.yawInternalState;
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        self.out.publish(pub); 
                     
    def step(self):
        if not self.config['Enable']: return
        self.stepSSbackward();         
        
if __name__ == "__main__":
    rospy.init_node("pladypos_dp");
    dp = DynamicPositioning();

    rate = rospy.Rate(10.0);
        
    rospy.spin();
    #while not rospy.is_shutdown():
        #dp.step();
        #rate.sleep();
        
        
        