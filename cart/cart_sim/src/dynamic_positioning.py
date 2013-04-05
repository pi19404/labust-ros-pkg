#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import roslib; roslib.load_manifest("cart_sim");
import rospy;
from auv_msgs.msg import BodyVelocityReq;
from auv_msgs.msg import NavSts;
from std_msgs.msg import Byte
import numpy
import math
      
class DynamicPositioning:
    def __init__(self):
        #rospy.Subscriber("desiredPosition", VehiclePose, self.onRef)
        rospy.Subscriber("stateHat", NavSts, self.onStateHat);
        rospy.Subscriber("trackedNav", NavSts, self.onTrackedNav);
        rospy.Subscriber("vcWindupFlag", Byte, self.onWindupFlag);
               
        self.stateHat = NavSts();
        
        self.out = rospy.Publisher("nuRef", BodyVelocityReq);
         
        w = numpy.array(rospy.get_param(
                            "dynamic_positioning/closed_loop_freq"));
        
        self.Kp = numpy.array([[2*w[0], 0],[0, 2*w[1]]],
                              dtype=numpy.float32);
        self.Ki = numpy.array([[w[0]*w[0], 0], [0, w[1]*w[1]]], 
                               dtype=numpy.float32);
                               
        self.Kt= 2*self.Ki;
        
        #self.Kp = numpy.reshape(
        #            numpy.array(rospy.get_param("dynamic_positioning/Kp"),
        #                        numpy.float32), [2,2]);                
        #self.Ki = numpy.reshape(
        #            numpy.array(rospy.get_param("dynamic_positioning/Ki"),
        #                        numpy.float32), [2,2]);
        self.Ts = rospy.get_param("dynamic_positioning/Ts");
        
        self.mode = rospy.get_param("dynamic_positioning/mode");
        
        self.max = numpy.array(rospy.get_param("dynamic_positioning/max"), dtype=numpy.float32);
               
        self.internalState = numpy.array([0,0], numpy.float32);
        self.state = numpy.array([0,0], numpy.float32);
        self.desired = numpy.array([0,0], numpy.float32);
        self.ff = numpy.array([0,0], numpy.float32);
        self.windup = numpy.array([0,0], dtype=numpy.int8);
        self.lastW = numpy.array([0,0], dtype=numpy.int8);
        self.lastI = numpy.array([0,0], numpy.float32);
        
        
        self.R = numpy.identity(2, numpy.float32);
        self.uk_1 = 0;
        self.lastTime = rospy.Time.now();
        
        '''Backward suff'''
        self.lastP = numpy.array([0,0]);
        self.lastFF = numpy.array([0,0]);
        
        model_name = rospy.get_param("model_name");
        b = rospy.get_param(model_name+"/dynamics/damping");
        Betainv = numpy.array([[1.0/b[0],0],[0,1.0/b[1]]], dtype=numpy.float32);
        cp = numpy.cos(numpy.pi/4); sp = numpy.sin(numpy.pi/4);
        allocM = numpy.array([[cp,cp,-cp,-cp],
                              [sp,-sp,sp,-sp]]);
                                                           
        self.Bstar = 0.9*numpy.dot(Betainv,allocM);       
        self.tmax = 13/(2*cp);           
        
    def onWindupFlag(self,data):
        if self.mode == 0:
            self.windup[0] = data.data & 1;
            self.windup[1] = (data.data & 2)>>1;
        else:
            self.windup[0] = data.data & 1;
            self.windup[1] = (data.data & 2**5)>>6;

        
    def onStateHat(self,data):
        self.stateHat = data;
        self.state = numpy.array([data.position.north, data.position.east],
                                 dtype=numpy.float32);
        yaw = data.orientation.yaw;
        self.R = numpy.array([[math.cos(yaw),-math.sin(yaw)],
                              [math.sin(yaw),math.cos(yaw)]],numpy.float32);
        
    def onRef(self,data):
        self.desired = numpy.array([data.position.north,data.position.east], 
                                   dtype=numpy.float32);                           
        self.step();                           
    
    def onTrackedNav(self,data):       
        self.ff[0] = data.body_velocity.x*math.cos(data.orientation.yaw);
        self.ff[1] = data.body_velocity.x*math.sin(data.orientation.yaw);
        self.onRef(data);
        
    def sat(self,data,minVal,maxVal):
        if data>maxVal: return maxVal;
        if data<minVal: return minVal;
        return data;
    
    def min(self,data1,data2):
        if data1<=data2: return data1;
        else: return data2;
        
    def max(self,data1,data2):
        if data1>=data2: return data1;
        else: return data2;
                
    def stepSSbackward(self):
        d = self.desired - self.state;
        
        #ti = numpy.dot(numpy.linalg.pinv(numpy.dot(numpy.linalg.inv(self.Kp),self.Bstar)),d);
        #print "Desired error:",d
        #print "Desired forces:",ti      
        #scale = numpy.max(numpy.array([numpy.abs(ti[0])/self.tmax,numpy.abs(ti[1])/self.tmax,1]));
        #ti = ti/scale;
        #print "Scaled forces:",ti
        #dd = numpy.dot(numpy.dot(numpy.linalg.inv(self.Kp),self.Bstar),ti);
        #print "Scaled error:",dd
        
        #d=dd;
        
        self.internalState += numpy.dot(self.Kp,d) - self.lastP;
        self.lastP = numpy.dot(self.Kp,d);
        
        self.internalState += numpy.dot(self.R,self.ff) - self.lastFF;
        self.lastFF = numpy.dot(self.R,self.ff);
        
        print "Feed forward:",self.lastFF;
        
        if (numpy.linalg.norm(self.lastW, 2) == 0) and (numpy.linalg.norm(self.windup, 2) != 0):
            print "Oduzimanje."
            self.internalState -= self.lastI;
            self.lastI = 0;
        
        if numpy.linalg.norm(self.windup, 2) == 0:
            print "Dodavanje"
            self.internalState += numpy.dot(self.Ki,d)*self.Ts;          
            self.lastI += numpy.dot(self.Ki,d)*self.Ts;
        
        self.lastW = 1*self.windup;
        
        u = numpy.dot(numpy.transpose(self.R),self.internalState);
        
        du = numpy.array([self.stateHat.body_velocity.x, self.stateHat.body_velocity.y], dtype=numpy.float32);
        #du = numpy.dot(self.R,du);
        
        #ti = numpy.dot(numpy.linalg.pinv(self.Bstar),u);
        #print "Desired speed:",u
        #print "Desired forces:",ti      
        #scale = numpy.max(numpy.array([numpy.abs(ti[0])/self.tmax,numpy.abs(ti[1])/self.tmax,1]));
        #ti = ti/scale;
        #print "Scaled forces:",ti
        #ddu = numpy.dot(self.Bstar,ti);
        #print "Scaled speed:",ddu
        
        #if (ddu[0] != u[0]) or (ddu[1] != u[1]):
                #if numpy.linalg.norm(self.windup, 2):
                #    self.internalState = numpy.dot(numpy.linalg.inv(numpy.identity(2, dtype=numpy.float32)+self.Kt*self.Ts),(self.internalState + 
                #                  numpy.dot(self.R,numpy.dot(self.Kt,ddu)*self.Ts) + numpy.dot(self.R,numpy.dot(self.Kt,du)*self.Ts)));
        #self.internalState = numpy.dot(numpy.linalg.inv(numpy.identity(2, dtype=numpy.float32)+self.Kt*self.Ts),(self.internalState + 
        #                          numpy.dot(self.R,numpy.dot(self.Kt,ddu)*self.Ts)));        
        
        print "Windup:",self.windup, u;
         
        '''The velocity only stuff'''                         
        #if numpy.linalg.norm(self.windup, 2):
        #    self.internalState = numpy.dot(numpy.linalg.inv(numpy.identity(2, dtype=numpy.float32)+self.Kt*self.Ts),(self.internalState + 
        #                          numpy.dot(self.Kt,du)*self.Ts));
        
        u = numpy.dot(numpy.transpose(self.R),self.internalState);
              
        #u = numpy.dot(numpy.transpose(self.R),ddu);                           
        pub = BodyVelocityReq();
        pub.twist.linear.x = u[0];
        pub.twist.linear.y = u[1];
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.yaw = pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub); 
        
        
    def stepSS(self):
        d = self.desired - self.state;
        u = numpy.dot(numpy.transpose(self.R),(numpy.dot(self.Kp,d) +
                                     self.internalState + self.ff));
                                  
        ti = numpy.dot(numpy.linalg.pinv(self.Bstar),u);
        print "Desired speed:",u
        print "Desired forces:",ti      
        scale = numpy.max(numpy.array([numpy.abs(ti[0])/self.tmax,numpy.abs(ti[1])/self.tmax,1]));
        ti = ti/scale;
        print "Scaled forces:",ti
        ddu = numpy.dot(self.Bstar,ti);
        print "Scaled speed:",ddu
        
        '''Ignore scaling'''
        ddu = u;
                                             
        #ddu = numpy.dot(numpy.transpose(self.R),numpy.array([1.0,1.0], dtype=numpy.float32));
        #ddu[0] = u[0] / scale;
        #ddu[1] = u[1] / scale;
        
        du = numpy.array([self.stateHat.body_velocity.x, self.stateHat.body_velocity.y], dtype=numpy.float32);
        #ddu = u;
                                                                                                  
        '''Propagate integration after the output calculation'''
        #if numpy.linalg.norm(du,2) == 0 :
        if numpy.linalg.norm(self.windup, 2) == 0:
            #self.internalState += self.windup*numpy.dot(self.R,numpy.dot(self.Kt,du-u)*self.Ts);
            #self.internalState += numpy.dot(self.R,numpy.dot(self.Kt,du-u)*self.Ts);
            #if self.windup[0]: u[0] = du[0];
            #if self.windup[1]: u[1] = du[1];     
            self.internalState += numpy.dot(self.Ki,d)*self.Ts + 0*numpy.dot(self.R,numpy.dot(self.Kt,ddu-u)*self.Ts);
        
        '''Ignore scaling'''
        #self.internalState += numpy.dot(self.Ki,d)*self.Ts + numpy.dot(self.R,numpy.dot(self.Kt,ddu-u)*self.Ts);
        
        
        #     print "No windup"
        #else:    
        self.uk_1 = u;
        
        print "Windup:",self.windup, u;
        #u=ddu;
        
        pub = BodyVelocityReq();
        pub.twist.linear.x = u[0];
        pub.twist.linear.y = u[1];
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.yaw = pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub);
        
    def stepSH(self):
        d =self.desired - self.state;
        
        u = 0.1*numpy.dot(self.Kp,d);
        yaw_ref = math.atan2(d[1],d[0]);       
                
        yaw_error = (yaw_ref - self.stateHat.orientation.yaw)%(2*math.pi);
        
        if yaw_error >= math.pi:
            yaw_error += -2*math.pi;
        elif yaw_error <= -math.pi:
            yaw_error += 2*math.pi;           

        pub = BodyVelocityReq();
        pub.twist.linear.x = u[0]*math.cos(yaw_ref) + u[1]*math.sin(yaw_ref);
        pub.twist.angular.z = 0.3*self.Kp[1][1]*yaw_error;
                
        '''Propagate integration after the output calculation'''
        #self.internalState[0] += numpy.dot(self.Ki,d)*self.Ts + self.windup[0]*self.Kt(self.stateHat.body_velocity.x - pub.twist.linear.x)*self.Ts;
        #self.internalState[1] += self.Ki[1][1]*yaw_error*self.Ts + self.windup[1]*self.Kt[1][1]*(self.stateHat.orientation_rate.yaw - pub.twist.angular.z)*self.Ts;
        
        print "Windup:",u, pub.twist.linear.x, pub.twist.angular.z
        
        pub.goal.priority = pub.goal.PRIORITY_NORMAL;
        pub.disable_axis.y = pub.disable_axis.pitch = 0;
        pub.disable_axis.roll = pub.disable_axis.z = 0;
        self.out.publish(pub);
        
    def step(self):
        self.Ts = (rospy.Time.now() - self.lastTime).to_sec();
        self.lastTime = rospy.Time.now();
        
        if self.Ts > 0.2: self.Ts = 0.1;
        self.Ts = 0.1;
        
        #rospy.loginfo("DynamicPositioning: sampling time %f",self.Ts);
        
        if self.mode == 0:
            self.stepSSbackward();
        else:
            self.stepSH();
         
        
if __name__ == "__main__":
    rospy.init_node("dpcontrol");
    dp = DynamicPositioning();
    
    synced = True;
        
    rate = rospy.Rate(10.0);
        
    while not rospy.is_shutdown():
        if synced:
            dp.step();
            rate.sleep();
        else:
            rospy.spin();
        
        
        