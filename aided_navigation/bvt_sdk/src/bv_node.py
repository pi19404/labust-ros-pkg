#!/usr/bin/env python
'''
 Software License Agreement (BSD License)

 Copyright (c) 2010, LABUST, UNIZG-FER
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  * Neither the name of the LABUST nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.

 Created on Feb 24, 2013
 Author: Dula Nad
'''
import rospy
import ctypes
import cv2
import numpy
from aidnav_msgs.msg import MBSonar

class BvNode:       
    def __init__(self):
        '''
        Initialize the sonar head from ROS parameters.
        '''
        
        locationType = ctypes.c_char_p(rospy.get_param("~type"));
        location = ctypes.c_char_p(rospy.get_param("~location"));
        headNum = rospy.get_param("~head",-1);
        self._rate = rospy.get_param("~rate",10);
               
        self._sonar = bvtsdk.BVTSonar_Create()
        bvtsdk.BVTSonar_Open(self._sonar,locationType,location);
        
        self._enumHeads(headNum);  
        
        self._pings = [];
        self._magnitudes = [];
        
        self._info();
          
    def _info(self):
        '''
        Output general configuration information
        for user information.
        ''' 
        
        print "Sonar.";
            
    def _enumHeads(self, headNum):        
        self._headCount=bvtsdk.BVTSonar_GetHeadCount(self._sonar);
        self._heads = [];
        self._headPubs = [];
        if headNum == -1:
            for i in range(self._headCount):
                head = ctypes.c_void_p(0);
                bvtsdk.BVTSonar_GetHead(
                        self._sonar, i,
                        ctypes.pointer(head));
                self._heads.append(head);
                self._headPubs.append(
                    rospy.Publisher("head"+str(i),MBSonar));
        else:
            head = ctypes.c_void_p(0);
            bvtsdk.BVTSonar_GetHead(
                self._sonar, headNum,
                ctypes.pointer(head));
            self._heads.append(head);
            self._headPubs.append(
                    rospy.Publisher("head"+str(i),MBSonar));
                   
    def _getPings(self):
        '''Destroy old pings.'''
        for ping in self._pings:
            bvtsdk.BVTPing_Destroy(ping);
        
        self._pings = []
        '''Acquire new pings.'''
        for head in self._heads:
            ping = ctypes.c_void_p(0);
            print "Acquire:", bvtsdk.BVTHead_GetPing(head,
                                   self._nextPing(), 
                                   ctypes.pointer(ping))
            self._pings.append(ping);
            
    def _nextPing(self):
        if (self._isFile):
            self._nextPing_
            
    def _getMagnitude(self):
        '''Destroy old pings.'''
        for image in self._magnitudes:
            bvtsdk.BVTMagImage_Destroy(image);
            
        self._magnitudes = [];
        '''Beamform new images.'''
        for ping in self._pings:
            image = ctypes.c_void_p(0);
            bvtsdk.BVTPing_GetImageRTheta(ping,
                                   ctypes.pointer(image))
            self._magnitudes.append(image);
            
    def publish(self):
        width = bvtsdk.BVTMagImage_GetWidth(self._magnitudes[0]);
        height = bvtsdk.BVTMagImage_GetHeight(self._magnitudes[0]);
        print "Image size:",width,"x",height
        data = numpy.zeros([height,width],dtype=numpy.uint16);
        datap = data.ctypes.data_as(ctypes.POINTER(ctypes.c_uint16));
        
        print "Error:",bvtsdk.BVTMagImage_CopyBits(
            self._magnitudes[0],
            datap,width*height)
               
        cv2.imshow("test", 50*data);
        cv2.waitKey(10);
        
        #i=0;
        #for image in self._magnitudes:
        #    msg = MBSonar();
        #    self._headPubs[i].publish(msg);
        #    i+=1;
              
    
                   
if __name__ == "__main__":
    rospy.init_node("bv_node");
     
    bvtsdk = ctypes.CDLL(rospy.get_param("~libName","libbvtsdk.so"));
    node = BvNode();
    
    rate = rospy.Rate(node._rate);
    
    while not rospy.is_shutdown():
        node._getPings();
        node._getMagnitude();
        node.publish();
        rate.sleep();
        
        