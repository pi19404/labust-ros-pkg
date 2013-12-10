#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import BodyForceReq;
from auv_msgs.msg import NavSts;
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from cart2.msg import ImuInfo
from cart2.msg import HLMessage
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
        self.states = OrderedDict.fromkeys(self.names, [0.0 for elem in logOrder]);
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
        self.pladyposLogOrder = ('current0',
                'current1',
                'current2',
                'current3',
                'current',
                'voltage',
                'rev0',
                'rev1',
                'rev2',
                'rev3');
        self.imusensLogOrder= ('linear_acceleration.x',
                               'linear_acceleration.y',
                               'linear_acceleration.z',
                               'angular_velocity.x',
                               'angular_velocity.y',
                               'angular_velocity.z',
                               'orientation.x',
                               'orientation.y',
                               'orientation.z',
                               'orientation.w');
        self.imuLogOrder = ('latDeg',
            'latFrac', 'lonDeg', 'lonFrac','status',
            'sog','cog','declination',
            'accel_x', 'accel_y', 'accel_z',
            'gyro_x', 'gyro_y', 'gyro_z',
            'mag_x', 'mag_y', 'mag_z',
            'roll','pitch','yaw',
            'ry','mmm','mm');
        self.hlDiagnostics = ('mode',
                            'ref_point.point.x',
                            'ref_point.point.y',
                            'ref_point.point.z',
                            'radius',
                            'surge',
                            'yaw')     
	self.acousticLogOrder = ('DistanceToPath',
		'DirectionToTarget',
		'RabbitNorth',
		'RabbitEast',
		'objects0zPos',
		'objects0xPos',
		'objects6zPos',
		'objects6xPos',
		'WPnorth1',
		'WPeast1',
		'WPnorth2',
		'WPeast2',
		'NonLinearCoeff',
		'TaskMode',
		'PathVelocity',
		'SpeedProfile',
		'RabbitDistance',
		'NonLinearCoeffDist',
		'Ki',
		'Sound');
        self.pointLogOrder = ('point.x','point.y','point.z')
        isPladypos = rospy.get_param("~use_pladypos",False);             

	loggerOrder=self.cart2LogOrder;
	if isPladypos:
		loggerOrder=self.pladyposLogOrder;
		
        self.loggers=[MessageLogger("logger/stateNames", NavSts, self.navStsLogOrder),
                      MessageLogger("logger/bodyVelReqNames", BodyVelocityReq, self.velLogOrder),
                      MessageLogger("logger/bodyForceReqNames", BodyForceReq, self.forceLogOrder),
                      MessageLogger("logger/HLDiagnostics", HLMessage, self.hlDiagnostics),
                      MessageLogger("logger/twistMessages", TwistStamped, self.velLogOrder),
                      MessageLogger("logger/Points", PointStamped, self.pointLogOrder),
                      MessageLogger("logger/imu", Imu, self.imusensLogOrder),
                      ListLogger("logger/cart2_info", ImuInfo, loggerOrder),
                      ListLogger("logger/imu_info", ImuInfo, self.imuLogOrder),
                      ListLogger("logger/acoustic_info", Float32MultiArray, self.acousticLogOrder)];
                             
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
        
        
        
