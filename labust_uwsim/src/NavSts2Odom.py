#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import NavSts;
from nav_msgs.msg import Odometry
import tf
import math
      
class MessageTransformer:
    def __init__(self):
        rospy.Subscriber("stateHat", NavSts, self.onNavSts);
        self.pub = rospy.Publisher("uwsim_hook", Odometry);
        self.listener = tf.TransformListener();
        self.broadcaster = tf.TransformBroadcaster();
        self.base_frame = rospy.get_param("~base_frame","base_link");
           
    def onNavSts(self,data):   
        q = tf.transformations.quaternion_from_euler(math.pi, 0, math.pi/2)
        self.broadcaster.sendTransform((0,0,0), q.conjugate(), rospy.Time.now(), "uwsim_frame", "local");
        
        try:
            (trans,rot) = self.listener.lookupTransform("uwsim_frame", self.base_frame, rospy.Time(0))
            
            odom = Odometry();  
            odom.twist.twist.linear.x = data.body_velocity.x;
            odom.twist.twist.linear.y = data.body_velocity.y;
            odom.twist.twist.linear.z = data.body_velocity.z;
            odom.twist.twist.angular.x = data.orientation_rate.roll;
            odom.twist.twist.angular.y = data.orientation_rate.pitch;
            odom.twist.twist.angular.z = data.orientation_rate.yaw;
            odom.child_frame_id = self.base_frame; 
            odom.pose.pose.orientation.x = rot[0];
            odom.pose.pose.orientation.y = rot[1];
            odom.pose.pose.orientation.z = rot[2];
            odom.pose.pose.orientation.w = rot[3];
            odom.pose.pose.position.x = trans[0];
            odom.pose.pose.position.y = trans[1];
            odom.pose.pose.position.z = trans[2];
            odom.header.stamp = rospy.Time.now();
            odom.header.frame_id = "local";
            
            self.pub.publish(odom);
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            None

if __name__ == "__main__":
    rospy.init_node("navsts2UWSim");
    tr = MessageTransformer();
    rospy.spin();
        
        