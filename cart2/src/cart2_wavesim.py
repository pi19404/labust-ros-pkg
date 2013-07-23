#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
import math;
from auv_msgs.msg import NavSts;
from std_msgs.msg import String;  
              
class WaveSim:
    def __init__(self):
        self.timebase = 0;
        self.Ts=0.1;
        self.H=rospy.get_param("wavesim/wave_height",4);
        self.T=rospy.get_param("wavesim/wave_period",5.6);
        self.L=rospy.get_param("wavesim/wave_length",50);
        self.k=2*math.pi/self.L;
        self.w=2*math.pi/self.T;
                 
    def onMeas(self, data):
        wave=String();
        self.timebase=self.timebase+self.Ts;
        theta=-self.k*data.position.north-self.w*self.timebase;
        elev=self.H/2*math.cos(theta);
        hv=math.pi*self.H/self.T*math.exp(self.k*elev)*math.cos(theta);
        wave.data=" ".join([str(-hv),'0','0']);
        wave_pub.publish(wave);         

if __name__ == "__main__":
    rospy.init_node("cart2_wavesim");  
    
    sim = WaveSim();
    
    rospy.Subscriber("meas_sim", NavSts, sim.onMeas)
    wave_pub = rospy.Publisher("currents",String)
      
    while not rospy.is_shutdown():
        rospy.spin();
        
        
        