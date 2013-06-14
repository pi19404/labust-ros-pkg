#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from geometry_msgs.msg import PointStamped;
from visualization_msgs.msg import Marker;       
                           
def onPoint(data):
    marker = Marker();
    marker.header.frame_id = "/cart2/local";
    marker.header.stamp = rospy.Time();
    marker.ns = "cart2";
    marker.id = 0;
    marker.type = Marker.SPHERE;
    marker.action = Marker.MODIFY;
    marker.pose.position.x = data.point.x;
    marker.pose.position.y = data.point.y;
    marker.pose.position.z = data.point.z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    vis_pub.publish(marker);                           

if __name__ == "__main__":
    rospy.init_node("cart2_rviz");

    rospy.Subscriber("target_point", PointStamped, onPoint)
    vis_pub = rospy.Publisher("visualization_marker",Marker)
    
    while not rospy.is_shutdown():
        rospy.spin();
        
        
        