#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import BodyForceReq;
from auv_msgs.msg import NavSts;
import functools
      
class MatLogger:
    def __init__(self):    
        self.varname={"stateHat":[],
                 "meas":[],
                 "usblEstimate":[],
                 "usblMeas":[],
                 "trajectory":[]};
                     
        for key in self.varname.keys():
            rospy.Subscriber(key, NavSts,
                             functools.partial(self.onNavSts,key));

        rospy.Subscriber("nuRef", BodyVelocityReq, self.onNuRef);
        rospy.Subscriber("tauOut", BodyForceReq, self.onTau);
        
        self.stateFile = open("state_log.csv",'w');
        self.nuRef = [];
          
    def onNavSts(self,name,data):
        self.varname[name]=[data.position.north,
           data.position.east,
           data.position.depth,
           data.orientation.roll,
           data.orientation.pitch,
           data.orientation.yaw,
           data.body_velocity.x,
           data.body_velocity.y,
           data.body_velocity.z,
           data.orientation_rate.roll,
           data.orientation_rate.pitch,
           data.orientation_rate.yaw];
                
    def onNuRef(self,data):
        self.nuRef = [data.twist.linear.x,
                      data.twist.linear.y,
                      data.twist.linear.z,
                      data.twist.angular.x,
                      data.twist.angular.y,
                      data.twist.angular.z];
        
    def onTau(self,data):
        self.tauRef = [data.wrench.force.x,
                       data.wrench.force.y,
                       data.wrench.force.z,
                       data.wrench.torque.x,
                       data.wrench.torque.y,
                       data.wrench.torque.z];
                       
        out = [rospy.Time.now().to_sec()] + self.tauRef + self.nuRef;
        for key in self.varname.keys():
            out += self.varname[key];
        self.stateFile.write(str(out).strip("[]") + "\n")
        
if __name__ == "__main__":
    rospy.init_node("dpcontrol");
    log = MatLogger(); 
        
    while not rospy.is_shutdown():
        rospy.spin();
        
        
        
        