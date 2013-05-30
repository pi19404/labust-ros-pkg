#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import BodyForceReq;
from auv_msgs.msg import NavSts;
from cart2.msg import ImuInfo
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
    def __init__(self, names, Type, logOrder):
        self.names = rospy.get_param(names);
        self.states = OrderedDict.fromkeys(self.names, Type());
        self.Type = Type;
        self.stateMux = Lock();
        self.logOrder=logOrder;
           
    def subscribe(self):
        for key in self.states.keys():
            rospy.Subscriber(key, self.Type,
                             functools.partial(self.onState,key));
    
    def onState(self,name,data):
        self.stateMux.acquire();
        self.states[name] = data;
        self.stateMux.release();
        
    def getHeader(self):
        retval=[];
        for name in self.names:
            retval.append("%element: "+name+" \n");
            retval.extend(["% " + elem + "\n" for elem in self.logOrder]);

        return retval;
                
    def getLogLine(self):
        self.stateMux.acquire();
        retval = ["," + ",".join(dataToStrList(self.states[name], self.logOrder))  for name in self.names];
        self.stateMux.release();
        return retval;
    
class ListLogger:
    def __init__(self, names, Type, logOrder):
        self.names = rospy.get_param(names);
        self.states = OrderedDict.fromkeys(self.names, []);
        self.Type = Type;
        self.stateMux = Lock();
        self.logOrder=logOrder;
           
    def subscribe(self):
        for key in self.states.keys():
            rospy.Subscriber(key, self.Type,
                             functools.partial(self.onState,key));
    
    def onState(self,name,data):
        self.stateMux.acquire();
        self.states[name] = data.data;
        self.stateMux.release();
        
    def getHeader(self):
        retval=[];
        for name in self.names:
            retval.append("%element: "+name+" \n");
            retval.extend(["% " + elem + "\n" for elem in self.logOrder]);

        return retval;
                
    def getLogLine(self):
        self.stateMux.acquire();
        retval = ["," + ",".join([str(elem) for elem in self.states[name]]) for name in self.names];
        self.stateMux.release();
        return retval;
      
class MatLogger:
    def __init__(self):    
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
        self.cart2LogOrder = ('port_rpm_desired',
                'stbd_rpm_desired',
                'port_rpm_meas',
                'stbd_rpm_meas',
                'port_curr_desired',
                'stbd_curr_desired',
                'current',
                'temp',
                'voltage');
        self.imuLogOrder = ('time',
            'lat', 'lon', 'hdop',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'mag_x', 'mag_y', 'mag_z',
            'roll','pitch','yaw',
            'modul','ry','mmm','mm');        
        
        self.loggers=[MessageLogger("logger/stateNames", NavSts, self.navStsLogOrder),
                      MessageLogger("logger/bodyVelReqNames", BodyVelocityReq, self.velLogOrder),
                      MessageLogger("logger/bodyForceReqNames", BodyForceReq, self.forceLogOrder),
                      ListLogger("logger/cart2_info", ImuInfo, self.cart2LogOrder),
                      ListLogger("logger/imu_info", ImuInfo, self.imuLogOrder)];
                             
        from datetime import datetime;
        name = rospy.get_param("~filename","log");
        dir = rospy.get_param("~dir",".");             
        self.logFile = open(dir + "/" + name + "_" + datetime.today().isoformat() + ".csv",'w');
                            
        for logger in self.loggers: 
            logger.subscribe();
               
        self.writeHeader();
                 
    def writeHeader(self):
        self.logFile.write("%The following element order can be found:\n");
        self.logFile.write("%element: stamp \n% t \n");
        for logger in self.loggers:
            self.logFile.writelines(logger.getHeader());                          
                           
    def start(self):
        rate = rospy.Rate(10);
        
        while not rospy.is_shutdown():
            self.logFile.write(str(rospy.Time.now().to_sec()));     
            for logger in self.loggers:
                self.logFile.writelines(logger.getLogLine());
            
            self.logFile.write("\n");            
            rate.sleep();
            
    def stop(self):
        self.logFile.close();
        
                           
if __name__ == "__main__":
    rospy.init_node("logger");
    log = MatLogger(); 
    log.start();
    while not rospy.is_shutdown():
        rospy.spin();
    log.stop();
        
        
        