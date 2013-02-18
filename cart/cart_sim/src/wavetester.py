#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from geometry_msgs.msg import Quaternion;
from nav_msgs.msg import Odometry
      
class Wave:
    def __init__(self):
        rospy.Subscriber("/diver/ocean_info", Quaternion, self.onWave);
        self.pub = rospy.Publisher("/diver/uwsim_hook",Odometry);
          
    def onWave(self,data):
        odom = Odometry();
        odom.pose.pose.position.x = 0;
        odom.pose.pose.position.y = 0;
        odom.pose.pose.position.z = -data.w;
        odom.pose.pose.orientation.w = 1;
        odom.header.stamp = rospy.Time.now();
         
        #print "Publish data:",data.w;
        self.pub.publish(odom);
                
    def step(self):
        self;
        
if __name__ == "__main__":
    rospy.init_node("wavetest");
    test = Wave();
    rate = rospy.Rate(1); 
        
    while not rospy.is_shutdown():
        test.step();
        rate.sleep();
        
        
        
        
        