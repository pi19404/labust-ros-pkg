#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import BodyForceReq;
from auv_msgs.msg import NavSts;
from threading import Lock
import functools

#\todo Add this to useful tools module in python
#\todo Add imu message, GPS message logging, imu raw logging
def dataToList(data, logOrder): 
    return [str(reduce(getattr, elem.split("."), data))
            for elem in logOrder];
      
class MatLogger:
    def __init__(self):    
        self.stateNames = rospy.get_param("logger/stateNames");
        self.bodyVelReqNames = rospy.get_param("logger/bodyVelReqNames");
        self.forceReqNames = rospy.get_param("logger/bodyForceReqNames");
        
        self.states = dict.fromkeys(self.stateNames, NavSts());
        self.velReq = dict.fromkeys(self.bodyVelReqNames, BodyVelocityReq());
        self.forceReq = dict.fromkeys(self.forceReqNames, BodyForceReq());
        
        from datetime import datetime;
        name = rospy.get_param("~filename","log");             
        self.logFile = open(name + "_" + datetime.today().isoformat() + ".csv",'w');
                            
        for key in self.states.keys():
            rospy.Subscriber(key, NavSts,
                             functools.partial(self.onNavSts,key));
        
        for key in self.velReq.keys():
            rospy.Subscriber(key, BodyVelocityReq,
                             functools.partial(self.onBodyVelReq,key));
                             
        for key in self.forceReq.keys():
            rospy.Subscriber(key, BodyForceReq,
                             functools.partial(self.onBodyForceReq,key));
        
        self.stateMux = Lock();
        self.forceMux = Lock();
        self.velMux = Lock();
        
        self.navStsLogOrder = (
                               'origin.latitude',
                               'origin.longitude',
                               'global_position.latitude',
                               'global_position.longitude',
                               'body_velocity.x',
                               'body_velocity.y',
                               'body_velocity.z',
                               'orientation_rate.roll',
                               'orientation_rate.pitch',
                               'orientation_rate.yaw',
                               'position.north',
                               'position.east',
                               'position.depth',
                               'orientation.roll',
                               'orientation.pitch',
                               'orientation.yaw');
        self.velLogOrder = (
                            'twist.linear.x',
                            'twist.linear.y',
                            'twist.linear.z',
                            'twist.angular.x',
                            'twist.angular.y',
                            'twist.angular.z');
        self.forceLogOrder = (
                            'wrench.force.x',
                            'wrench.force.y',
                            'wrench.force.z',
                            'wrench.torque.x',
                            'wrench.torque.y',
                            'wrench.torque.z');
        
        self.writeHeader();
          
    def onNavSts(self,name,data):
        self.stateMux.acquire();
        self.states[name] = data;
        self.stateMux.release();
        
    def writeHeader(self):
        self.logFile.write("%The following element order can be found:\n");
        i = 1;
        self.logFile.write("% "+ str(i)+ ". Time \n");
        
        for name in self.stateNames:
            self.logFile.write("% "+name+" elements \n");
            for elem in self.navStsLogOrder:
                i+=1;
                self.logFile.write("%  " + str(i) + "." + elem + "\n");
                
        for name in self.bodyVelReqNames:
            self.logFile.write("% "+name+" elements \n");
            for elem in self.velLogOrder:
                i+=1;
                self.logFile.write("%  " + str(i) + "." + elem + "\n");
                
        for name in self.forceReqNames:
            self.logFile.write("% "+name+" elements \n");
            for elem in self.forceLogOrder:
                i+=1;
                self.logFile.write("%  " + str(i) + "." + elem + "\n");                
        
        
          
#        return [data.body_velocity.x,
#           data.body_velocity.y,
#           data.body_velocity.z,
#           data.orientation_rate.roll,
#           data.orientation_rate.pitch,
#           data.orientation_rate.yaw,
#           data.position.north,
#           data.position.east,
#           data.position.depth,
#           data.orientation.roll,
#           data.orientation.pitch,
#           data.orientation.yaw];                   
                           
    def onBodyVelReq(self,name, data):
        print "New body velocity message: ",name;
        self.velMux.acquire();
        self.velReq[name] = data;
        self.velMux.release();
#        self.nuRef = [data.twist.linear.x,
#                      data.twist.linear.y,
#                      data.twist.linear.z,
#                      data.twist.angular.x,
#                      data.twist.angular.y,
#                      data.twist.angular.z];

    def onBodyForceReq(self,name, data):
        print "New body force message: ",name;
        self.forceMux.acquire();
        self.states[name] = data;
        self.forceMux.release();
        
#    def onTau(self,data):
#        self.tauRef = [data.wrench.force.x,
#                       data.wrench.force.y,
#                       data.wrench.force.z,
#                       data.wrench.torque.x,
#                       data.wrench.torque.y,
#                       data.wrench.torque.z];
#                       
#        out = [rospy.Time.now().to_sec()] + self.tauRef + self.nuRef;
#        out += self.varname["meas"] + self.varname["stateHat"];
#        out += self.varname["usblMeas"] + self.varname["usblEstimate"];
#        out += self.varname["/diver/meas"];
#        out += self.varname["trajectory"];
#        self.stateFile.write(str(out).strip("[]") + "\n")

    def start(self):
        rate = rospy.Rate(10);
        
        while not rospy.is_shutdown():
            self.stateMux.acquire();
            #print self.navStsToList(self.states['stateHat']);
            #self.logFile.write(','.join(self.navStsToList(self.states['stateHat']))) + "\n");
            self.stateMux.release();
            rate.sleep();
            
                
if __name__ == "__main__":
    rospy.init_node("logger");
    log = MatLogger(); 
    log.start();
    while not rospy.is_shutdown():
        rospy.spin();
        
        
        
        