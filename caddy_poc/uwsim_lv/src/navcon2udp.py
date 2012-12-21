'''
Created on Dec 19, 2012

@author: dnad
'''

import roslib; roslib.load_manifest("uwsim_lv");
import rospy;
from navigation_g500.msg import FastraxIt500Gps
from auv_msgs.msg import BodyForceReq
from geometry_msgs.msg import Vector3 
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion
import threading
import PyKDL
from numpy import array
import math

class NavHandler:
    """Handles incoming navigation messages and UDP communication"""
    def __init__(self):
        rospy.init_node("navcon2udp");
        '''Subscribe to sensor messages'''
        gpsName = rospy.get_param("~gpsName","navigation_g500/fastrax_it_500_gps");
        imuName = rospy.get_param("~imuName","navigation_g500/imu");
        
        rospy.Subscriber(gpsName, FastraxIt500Gps, self.onGps);
        rospy.Subscriber(imuName, Imu, self.onImu);
        
        '''Sensor transformation matrix'''
        self.imu_tf = self.computeTf(array(rospy.get_param("tritech_igc_gyro/tf")));
        self.dataMux=threading.Lock();
        self.runFlag=1;
        self.commsThread = threading.Thread(target=self.comms);
        self.commsThread.start();
        
        '''Initialize values'''
        self.roll=self.pitch=self.yaw=self.north=self.east=self.gpsValid=0;
        
    def onImu(self,data):
        roll,pitch,yaw = euler_from_quaternion(
            [data.orientation.x, 
            data.orientation.y,
            data.orientation.z,
            data.orientation.w]);
        imu_data =  PyKDL.Rotation.RPY(roll, pitch, yaw);
        imu_data = imu_data * self.imu_tf.M;
        self.dataMux.acquire(); 
        self.roll,self.pitch,self.yaw = imu_data.GetRPY();
        self.dataMux.release();        
        #print("Roll-pitch-yaw (%f,%f,%f)",self.roll,self.pitch,self.yaw);
        
    def onGps(self,data):
        self.dataMux.acquire(); 
        self.north = data.north;
        self.east = data.east;
        self.gpsValid=1;
        self.dataMux.release();
        #print("Received data (%f,%f).",data.north,data.east);   
        
    def computeTf(self, tf):
        r = PyKDL.Rotation.RPY(math.radians(tf[3]), math.radians(tf[4]), math.radians(tf[5]))
        v = PyKDL.Vector(tf[0], tf[1], tf[2])
        frame = PyKDL.Frame(r, v)
        return frame
    
    def comms(self):    
        '''Create TAU publisher''' 
        tauName = rospy.get_param("~tauName","control_g500/velocity_to_force_req");
        pub = rospy.Publisher(tauName, BodyForceReq)
    
        '''Open UDP socket'''
        import socket
        comms = socket.socket(socket.AF_INET, socket.SOCK_DGRAM);
        comms.bind(("127.0.0.1",rospy.get_param("/lv_host_port",33346)));

        while self.runFlag:
            '''Get and send TAU data'''
            data,addr = comms.recvfrom(100);
            #print("Recevied:%s",data);
            tau=double(data.split(","));      
            if len(tau) == 6:      
                pub.publish(BodyForceReq(wrench = Wrench(Vector3(tau[0],tau[1],tau[2]), Vector3(tau[3],tau[4],tau[5]))));
    
            self.dataMux.acquire();
            comms.sendto(",".join([self.gpsValid, str(self.north), str(self.east), str(self.roll), str(self.pitch), str(self.yaw)]),addr);
            self.gpsValid = 0;
            self.dataMux.release();          
        
    def stop(self):
        self.runFlag = 0;
        self.commsThread.join();
    
def navcon2udp():
    handler = NavHandler();
    '''Start ROS node.'''
    rospy.spin();
    handler.stop();        

if __name__ == '__main__':
    navcon2udp()