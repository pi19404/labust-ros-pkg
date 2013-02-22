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
      
class LawnGenerator:
    def __init__(self):
        self.out = rospy.Publisher("trajectory", NavSts);
        
        self.u = 0.5;
        self.Ts = 0.1;   
        
        self.points=[[0,0],[30,0],[30,30],[0,30]];
        self.lastPoint = numpy.array(self.points[0],dtype=numpy.float32);
        self.next = 1;
        self.nextPoint = numpy.array(self.points[self.next],
                                     dtype=numpy.float32);
        self.radius = 0.5;     
          
    def step(self):
        
        d = numpy.linalg.norm(self.lastPoint - self.nextPoint,2);
               
        if d < self.radius:
            self.next = (self.next+1) % len(self.points);
            self.nextPoint = numpy.array(self.points[self.next],
                                         dtype=numpy.float32);
            print "Change point to:",self.nextPoint;
            
        diff = self.nextPoint - self.lastPoint;
        yaw = math.atan2(diff[1], diff[0]);
        self.lastPoint += self.u*self.Ts*numpy.array(
                                            [math.cos(yaw),
                                             math.sin(yaw)]);
        
        pub = NavSts();
        pub.position.north = self.lastPoint[0];
        pub.position.east = self.lastPoint[1];
        self.out.publish(pub);
        
        #print "Last point:", self.lastPoint;
         
        
if __name__ == "__main__":
    rospy.init_node("lawnGenerator");
    dp = LawnGenerator();
    
    rate = rospy.Rate(10.0);
        
    while not rospy.is_shutdown():
        dp.step();
        rate.sleep();
        
        