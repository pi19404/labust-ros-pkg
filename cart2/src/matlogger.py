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
from collections import OrderedDict
import functools

#\todo Add this to useful tools module in python
#\todo Add imu message, GPS message logging, imu raw logging
def dataToList(data, logOrder): 
    return [reduce(getattr, elem.split("."), data)
            for elem in logOrder];
            
def dataToStrList(data, logOrder): 
    return [str(reduce(getattr, elem.split("."), data))
            for elem in logOrder];
            
class MessageLogger:
    def __init__(self, names, Type):
        self.names = rospy.get_param(names);
        self.states = OrderedDict.fromkeys(self.name, Type());
        self.Type = Type;
        self.stateMux = Lock();
        self.logOrder=[];
           
    def subscribe(self):
        for key in self.states.keys():
            rospy.Subscriber(key, self.Type,
                             functools.partial(self.onState,key));
    
    def onNavSts(self,name,data):
        self.stateMux.acquire();
        self.states[name] = data;
        self.stateMux.release();
        
    def getHeader(self):
        for name in self.stateNames:
            self.logFile.write("%element: "+name+" \n");
            for elem in self.navStsLogOrder:
                self.logFile.write("% " + elem + "\n");
                
    def getLogLine(self):
        for name in self.stateNames:
            self.logFile.write(",");
            self.logFile.write(",".join(dataToStrList(self.states[name], self.LogOrder)));
      
class MatLogger:
    def __init__(self):    
        self.stateNames = rospy.get_param("logger/stateNames");
        self.bodyVelReqNames = rospy.get_param("logger/bodyVelReqNames");
        self.forceReqNames = rospy.get_param("logger/bodyForceReqNames");
        
        self.loggers=[MessageLogger("logger/stateNames", NavSts),
                      MessageLogger("logger/bodyVelReqNames", BodyVelocityReq),
                      MessageLogger("logger/bodyForceReqNames", BodyForceReq)];
                      
        
        self.states = OrderedDict.fromkeys(self.stateNames, NavSts());
        self.velReq = OrderedDict.fromkeys(self.bodyVelReqNames, BodyVelocityReq());
        self.forceReq = OrderedDict.fromkeys(self.forceReqNames, BodyForceReq());
        
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
            self.logFile.write("%element: "+name+" \n");
            for elem in self.navStsLogOrder:
                i+=1;
                #self.logFile.write("%  " + str(i) + "." + elem + "\n");
                self.logFile.write("%" + elem + "\n");
                
        for name in self.bodyVelReqNames:
            i=0;
            self.logFile.write("%element: "+name+" \n");
            for elem in self.velLogOrder:
                i+=1;
                #self.logFile.write("%  " + str(i) + "." + elem + "\n");
                self.logFile.write("%  " + elem + "\n");
                
        for name in self.forceReqNames:
            i=0;
            self.logFile.write("%element: "+name+" \n");
            for elem in self.forceLogOrder:
                i+=1;
                #self.logFile.write("%  " + str(i) + "." + elem + "\n");   
                self.logFile.write("%  " + elem + "\n");                             
                           
    def onBodyVelReq(self,name, data):
        self.velMux.acquire();
        self.velReq[name] = data;
        self.velMux.release();

    def onBodyForceReq(self,name, data):
        self.forceMux.acquire();
        self.states[name] = data;
        self.forceMux.release();

    def start(self):
        rate = rospy.Rate(10);
        
        while not rospy.is_shutdown():
            self.stateMux.acquire();
            self.logFile.write(str(rospy.Time.now().to_sec()));     
            
            for name in self.stateNames:
                self.logFile.write(",");
                self.logFile.write(",".join(dataToStrList(self.states[name], self.navStsLogOrder)));
                      
            for name in self.bodyVelReqNames:
                self.logFile.write(",");
                self.logFile.write(",".join(dataToStrList(self.velReq[name], self.velLogOrder)));
                
            for name in self.forceReqNames:
                self.logFile.write(",");
                self.logFile.write(",".join(dataToStrList(self.forceReq[name], self.forceLogOrder)));
                
            self.logFile.write("\n");
            
            self.stateMux.release();
            rate.sleep();
            
                
if __name__ == "__main__":
    rospy.init_node("logger");
    log = MatLogger(); 
    log.start();
    while not rospy.is_shutdown():
        rospy.spin();
        
        
        
        