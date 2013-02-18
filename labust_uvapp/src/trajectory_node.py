#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("labust_uvapp");
import rospy;
from auv_msgs.msg import BodyVelocityReq;
import trajectory_generators as trg
      
class TrajectoryNode:
    def __init__(self):
        self.mode = rospy.get_param("trajectory_generator/mode");
        
        spoints = rospy.get_param("trajectory_generator/speed_points", [])      
                 
        self._speedGenX = trg.Speed(spoints[0]);
        self._speedGenY = trg.Speed(spoints[1]);
                    
        self._speedInit();
        self._lastTime = rospy.Time.now();
        '''self.register()
                
        
        self.u = 0.5;
        self.Ts = 0.1;   
        
        self.points=[[0,0],[50,0],[50,50],[0,50]];
        self.lastPoint = numpy.array(self.points[0],dtype=numpy.float32);
        self.next = 1;
        self.nextPoint = numpy.array(self.points[self.next],
                                     dtype=numpy.float32);
        self.radius = 0.5;'''
        
    def _speedInit(self):
        self._bspdreq = rospy.Publisher("nuRef",BodyVelocityReq);
                  
    def step(self):
        out = BodyVelocityReq();
        out.twist.linear.x = self._speedGenX.step((rospy.Time.now() - self._lastTime).to_sec());
        out.twist.linear.y = self._speedGenY.step((rospy.Time.now() - self._lastTime).to_sec());
        self._lastTime = rospy.Time.now();
        
        self._bspdreq.publish(out);
        
        '''
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
        
        #print "Last point:", self.lastPoint; '''
         
        
if __name__ == "__main__":
    rospy.init_node("trajectory_node");
    node = TrajectoryNode();
    
    rate = rospy.Rate(rospy.get_param("trajectory_generator/rate",10.0));
        
    while not rospy.is_shutdown():
        node.step();
        rate.sleep();
        
        