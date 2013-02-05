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
      
class MatLogger:
    def __init__(self):
        rospy.Subscriber("stateHat", NavSts, self.onNavSts);
        rospy.Subscriber("meas", NavSts, self.onNavSts);
        rospy.Subscriber("usblEstimate", NavSts, self.onNavSts);
        rospy.Subscriber("usblMeas", NavSts, self.onNavSts);
        rospy.Subscriber("trajectory", NavSts, self.onNavSts);
        rospy.Subscriber("nuRef", BodyVelocityReq, self.onNuRef);
        rospy.Subscriber("tauOut", BodyForceReq, self.onTau);
        
        self.stateFile = open("state_log.csv",'w');
        self.navTopics = dict();
        
    def onNavSts(self,data):
        print data._connection_header["topic"];
        self.navTopics['test'] = data;
        
    def onNuRef(self,data):
        self;
        
    def onTau(self,data):
        self;
                       
    def step(self):
        print self.navTopics.keys();
        
        
if __name__ == "__main__":
    rospy.init_node("dpcontrol");
    log = MatLogger();
        
    rate = rospy.Rate(10.0);
        
    while not rospy.is_shutdown():
        log.step();
        rate.sleep();
        
        
        