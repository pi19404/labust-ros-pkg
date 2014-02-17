#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import os, sys

import rospy
import labust_gui
import math

from python_qt_binding import loadUi,QtCore, QtGui
from qt_gui.plugin import Plugin

from std_msgs.msg import String
from auv_msgs.msg import Bool6Axis
from navcon_msgs.srv import EnableControl
from navcon_msgs.srv import ConfigureAxes
from labust_uvapp.srv import ConfigureVelocityController

class SimManGui(QtGui.QWidget):
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)
        
        self.manualSelection = 0;
        self.movebaseSelection = 2;
        
        pass

    def setup(self, name, ros):
        self.setObjectName(name)
        self.setWindowTitle(name)
        self._connect_signals(ros)
        self._booststrap()
        pass
    
    def unload(self):
        pass

    def onStateHat(self,data):
        self.statusNorth.setText("{:.2f}".format(data.position.north))
        self.statusEast.setText("{:.2f}".format(data.position.east))
        self.statusDepth.setText("{:.2f}".format(data.position.depth))
        hdg = math.degrees(data.orientation.yaw)
        if hdg < 0: hdg += 360
        self.statusHeading.setText("{:.2f}".format(hdg))
        self.statusAltitude.setText("{:.2f}".format(data.altitude))
        self.statusRoll.setText("{:.2f}".format(math.degrees(data.orientation.roll)))
        self.statusPitch.setText("{:.2f}".format(math.degrees(data.orientation.pitch)))
        
        self._update_refs(data)
        
        pass
    
    def _connect_signals(self, ros):
        QtCore.QObject.connect(self, QtCore.SIGNAL("onHeadingEnable"), ros.onHeadingEnable)  
        self.headingEnable.toggled.connect(lambda state: 
                                           self.emit(QtCore.SIGNAL("onHeadingEnable"), 
                                                     state, 
                                                     self.headingRef.value()))
        QtCore.QObject.connect(self, QtCore.SIGNAL("onManualEnable"), ros.onManualEnable)  
        self.manualEnable.toggled.connect(lambda state: 
                                            self.emit(QtCore.SIGNAL("onManualEnable"), 
                                                      state, self.manualSelection))
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDepthEnable"), ros.onDepthEnable)  
        self.depthEnable.toggled.connect(self._onDepthEnable)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onAltitudeEnable"), ros.onAltitudeEnable)  
        self.altitudeEnable.toggled.connect(self._onAltitudeEnable) 
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDPEnable"), ros.onDPEnable)  
        self.dpEnable.toggled.connect(self._onDpEnable)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onPitchEnable"), ros.onPitchEnable)
        self.pitchEnable.toggled.connect(lambda state: 
                                           self.emit(QtCore.SIGNAL("onPitchEnable"), 
                                           state, 
                                           self.pitchRef.value()))
                                       
        
        QtCore.QObject.connect(self, QtCore.SIGNAL("onHeadingUpdate"), ros.onHeadingUpdate)
        self.headingRef.editingFinished.connect(lambda: 
                                                    self.emit(QtCore.SIGNAL("onHeadingUpdate"), 
                                                    self.headingRef.value()))
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDepthUpdate"), ros.onDepthUpdate)
        self.depthRef.editingFinished.connect(lambda: 
                                                    self.emit(QtCore.SIGNAL("onDepthUpdate"), 
                                                    self.depthRef.value()))
        QtCore.QObject.connect(self, QtCore.SIGNAL("onAltitudeUpdate"), ros.onAltitudeUpdate)
        self.altitudeRef.editingFinished.connect(lambda: 
                                                    self.emit(QtCore.SIGNAL("onAltitudeUpdate"), 
                                                    self.altitudeRef.value()))
        
        QtCore.QObject.connect(self, QtCore.SIGNAL("onMoveBaseRef"), ros.onMoveBaseRef)
        
        self.Stop.clicked.connect(self._onStop)
       
        self.tauRadio.toggled.connect(self._onJoystickRadio)
        self.nuRadio.toggled.connect(self._onJoystickRadio)
        self.relRadio.toggled.connect(self._onJoystickRadio)
        
        self.absRadio.toggled.connect(self._onMoveBaseRadio)
        self.nePosRadio.toggled.connect(self._onMoveBaseRadio)
        self.relPosRadio.toggled.connect(self._onMoveBaseRadio)
        self.northRef.editingFinished.connect(self._onMoveBaseRadio)
        self.eastRef.editingFinished.connect(self._onMoveBaseRadio)

    def _onStop(self):
        self.dpEnable.setChecked(False)
        self.headingEnable.setChecked(False)
        self.depthEnable.setChecked(False)
        self.manualEnable.setChecked(False)
            
    def _onJoystickRadio(self):
        if self.tauRadio.isChecked(): self.manualSelection = 0;
        if self.nuRadio.isChecked(): self.manualSelection = 1;
        if self.relRadio.isChecked(): self.manualSelection = 2;
        self.emit(QtCore.SIGNAL("onManualEnable"), 
                  self.manualEnable.isChecked(), 
                  self.manualSelection)
    
    def _onMoveBaseRadio(self):
        if self.absRadio.isChecked(): self.movebaseSelection = 0;
        if self.nePosRadio.isChecked(): self.movebaseSelection = 1;
        if self.relPosRadio.isChecked(): self.movebaseSelection = 2;
        self.emit(QtCore.SIGNAL("onMoveBaseRef"), 
                  self.northRef.value(), 
                  self.eastRef.value(),
                  self.movebaseSelection)
        
    def _onDpEnable(self, state):
        self.emit(QtCore.SIGNAL("onDPEnable"), state)
        self.emit(QtCore.SIGNAL("onMoveBaseRef"),
                  self.northRef.value(),
                  self.eastRef.value(),
                  self.movebaseSelection)
        
    def _onAltitudeEnable(self, state):
        #disable depth
        if self.depthEnable.isChecked():
            self.emit(QtCore.SIGNAL("onDepthEnable"), False, 0)
            
        self.emit(QtCore.SIGNAL("onAltitudeEnable"), state, self.altitudeRef.value())
            
    def _onDepthEnable(self, state):
        #disable depth
        if self.altitudeEnable.isChecked():
            self.emit(QtCore.SIGNAL("onAltitudeEnable"), False, 0)
            
        self.emit(QtCore.SIGNAL("onDepthEnable"), state, self.depthRef.value())       
        
        
    def _update_refs(self,data):
        if not self.headingEnable.isChecked():
            hdg = math.degrees(data.orientation.yaw)
            if hdg < 0: hdg += 360
            self.headingRef.setValue(hdg)
        if not self.depthEnable.isChecked():
            self.depthRef.setValue(data.position.depth)
        if not self.pitchEnable.isChecked():
          self.pitchRef.setValue(math.degrees(data.orientation.pitch)) 
        if not self.altitudeEnable.isChecked():
            self.altitudeRef.setValue(data.altitude)
            
    def _booststrap(self):
        self.headingEnable.setChecked(False)
        self.depthEnable.setChecked(False)
    
    
from auv_msgs.msg import NavSts

class HLManager:
    def __init__(self):
        self.man_nu = Bool6Axis()
        self.man_nu.x = False
        self.man_nu.y = False
        self.man_nu.z = False
        self.man_nu.roll = True
        self.man_nu.pitch = True
        self.man_nu.yaw = False
        
        self.velcon = [0,0,0,0,0,0]
        self.manualEnable = 0
        self.manualSelection = 0
        
        self.hdgEnable = 0
        self.altEnable = 0
        self.depthEnable = 0
        self.dpEnable = 0
        self.pitchEnable = 0
             
        self.enablers={"HDG":"HDG_enable",
                       "ALT":"ALT_enable",
                       "DEPTH":"DEPTH_enable",
                       "DP":"FADP_enable",
                       "NU":"NU_enable",
                       "REF":"REF_enable",
                       "PITCH":"PITCH_enable"};
        for v in self.enablers.keys():
           self._invoke_enabler(v, False) 
                       
        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
        
    def manual_control(self, state, selection):
        """Turn on manual control for 
        uncontrolled axes"""
        self.manualEnable = state
        self.manualSelection = selection
        
        self._invoke_enabler("NU", False)
        self._invoke_enabler("REF", False)
        if state:
            if selection == 0:
                if not self.dpEnable:
                    self.velcon[0] = 1
                    self.velcon[1] = 1
                    
                if not self.altEnable: self.velcon[2] = 1
                if not self.hdgEnable: self.velcon[5] = 1
                if not self.pitchEnable: self.velcon[4] = 1
            elif selection == 1:
                for i,e in enumerate(self.velcon):
                    if (e != 2): self.velcon[i]=2
                self._invoke_enabler("NU", True)
            elif selection == 2:
                self._invoke_enabler("REF", True)
        else:
                for i,e in enumerate(self.velcon):
                    if (e == 1): self.velcon[i]=0
        
        #Roll and pitch are not controlled               
        self.velcon[3] = 0
        #self.velcon[4] = 0
        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
                     
    def heading_control(self, state):
        self.hdgEnable = state
        self._invoke_enabler("HDG", state)
        if state:
            self.man_nu.yaw = 1
            self.velcon[5] = 2
        else:
            self.man_nu.yaw = 0
            self.velcon[5] = 0
            self.manual_control(self.manualEnable, self.manualSelection)

        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
        
    def depth_control(self, state):
        if state and self.altEnable:
            #Disable altitude first
            self._invoke_enabler("ALT", False)
            self.altEnable = False;
        
        self.depthEnable = state
        self._invoke_enabler("DEPTH", state)
        if state:
            self.man_nu.z = 1
            self.velcon[2] = 2
        else:
            self.man_nu.z = 0
            self.velcon[2] = 0
            self.manual_control(self.manualEnable, self.manualSelection)

        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
        
    def altitude_control(self, state):
        if state and self.depthEnable:
            #Disable depth first
            self._invoke_enabler("DEPTH", False)
            self.depthEnable = False;
                        
        self.altEnable = state
        self._invoke_enabler("ALT", state)
        if state:
            self.man_nu.z = 1
            self.velcon[2] = 2
        else:
            self.man_nu.z = 0
            self.velcon[2] = 0
            self.manual_control(self.manualEnable, self.manualSelection)

        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
        
    def pitch_control(self, state):
        self.pitchEnable = state
        self._invoke_enabler("PITCH", state)
        if state:
            self.man_nu.pitch = 1
            self.velcon[4] = 2
        else:
            self.man_nu.pitch = 0
            self.velcon[4] = 0
            self.manual_control(self.manualEnable, self.manualSelection)

        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
        
    def dp_control(self, state):
        self.dpEnable = state
        self._invoke_enabler("DP", state)
        if state:
            self.man_nu.x = 1
            self.man_nu.y = 1
            self.velcon[0] = 2
            self.velcon[1] = 2
        else:
            self.man_nu.x = 0
            self.man_nu.y = 0
            self.velcon[0] = 0
            self.velcon[1] = 0
            self.manual_control(self.manualEnable, self.manualSelection)

        self._invoke_manual_nu_cfg()
        self._invoke_velcon_cfg()
         
    def _invoke_enabler(self, name, data):
        if self.enablers[name] == "None": return      
        try:
            rospy.wait_for_service(self.enablers[name], timeout=1.0)
            enabler = rospy.ServiceProxy(self.enablers[name], EnableControl)
            enabler(data)
        except (rospy.ROSException, rospy.ServiceException), e:
            print "Removing enabler from list."
            self.enablers[name] = "None"
            print "Service call failed: %s"%e
                  
    def _invoke_manual_nu_cfg(self):
        try:
            rospy.wait_for_service("ConfigureAxes")
            configurer = rospy.ServiceProxy("ConfigureAxes", ConfigureAxes)
            configurer(self.man_nu)
        except (rospy.ROSException, rospy.ServiceException), e:
            print "Service call failed: %s"%e
                 
    def _invoke_velcon_cfg(self):
        try:
            rospy.wait_for_service("ConfigureVelocityController")
            configurer = rospy.ServiceProxy("ConfigureVelocityController", ConfigureVelocityController)
            configurer(desired_mode=self.velcon)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e      
        

class SimManROS(QtCore.QObject):
        def __init__(self):
            QtCore.QObject.__init__(self)
            self._subscribers = []
            self._stateRef = NavSts()
            self._stateHat = NavSts()
            
            self.manager = HLManager()
                                          
        def setup(self, gui):
            self._subscribe()
            self._advertise()
            self._connect_signals(gui)
            pass
        
        def unload(self):
            self._unsubscribe()
            pass
        
        def onHeadingUpdate(self, heading):
            self._stateRef.orientation.yaw = math.radians(heading)
            self._update_stateRef()
            
        def onDepthUpdate(self, depth):
            self._stateRef.position.depth = depth
            self._update_stateRef()
            
        def onPitchUpdate(self, pitch):
            self._stateRef.orientation.pitch = math.radians(pitch)
            self._update_stateRef()
            
        def onAltitudeUpdate(self, altitude):
            self._stateRef.altitude = altitude
            self._update_stateRef()
            
        def onMoveBaseRef(self, x, y, selection):
            if selection == 0:
                self._stateRef.position.north = x
                self._stateRef.position.east = y
            elif selection == 1:
                self._stateRef.position.north = self._stateHat.position.north + x
                self._stateRef.position.east = self._stateHat.position.east + y
                #self.northRef.value(0.0)
                #self.eastRef.value(0.0)
            elif selection == 2:
                import numpy
                yaw = self._stateHat.orientation.yaw
                R = numpy.array([[math.cos(yaw),-math.sin(yaw)],
                              [math.sin(yaw),math.cos(yaw)]],numpy.float32)
                xy = numpy.array([x,y],numpy.float32)
                ne = numpy.dot(R,xy)
                self._stateRef.position.north = self._stateHat.position.north + ne[0]
                self._stateRef.position.east = self._stateHat.position.east + ne[1]
                #self.northRef.value(0.0)
                #self.eastRef.value(0.0)
                
            self._update_stateRef()
            
        def onHeadingEnable(self, state, heading):
            #Invoke the enable services
            self.manager.heading_control(state);
            self.onHeadingUpdate(heading)           
            
        def onDepthEnable(self, state, depth):
            #Invoke the enable services
            self.manager.depth_control(state);
            self.onDepthUpdate(depth)
            
        def onAltitudeEnable(self, state, altitude):
            #Invoke the enable services
            self.manager.altitude_control(state);
            self.onAltitudeUpdate(altitude) 
        
        def onPitchEnable(self, state, depth):
            #Invoke the enable services
            self.manager.pitch_control(state);
            self.onPitchUpdate(depth)  
            
        def onDPEnable(self, state):
            #Invoke the enable services
            self.manager.dp_control(state);
            
        def onManualEnable(self, state, selection):
            self.manager.manual_control(state, selection);
                             
        def _update_stateRef(self):
            self._stateRefPub.publish(self._stateRef)

        def _connect_signals(self, gui):
            QtCore.QObject.connect(self, 
                QtCore.SIGNAL("onStateHat"), gui.onStateHat)
            
        def _onStateHat(self, data):
            self._stateHat = data;
            self.emit(QtCore.SIGNAL("onStateHat"), data)
            
        def _subscribe(self):   
            self._subscribers.append(rospy.Subscriber("stateHat", 
                                                      NavSts, 
                                                      self._onStateHat))
        
        def _advertise(self):
            self._stateRefPub = rospy.Publisher("stateRef", NavSts)
        
        def _unsubscribe(self):
            for subscriber in self._subscribers:
                subscriber.unregister()
                
            self._stateRefPub.unregister()


class SimManRqt(Plugin):
    def __init__(self, context):
        name = "SimMan"
        resource = os.path.join(os.path.dirname(
                        os.path.realpath(__file__)), 
                        "resource/" + name + ".ui")
        GuiT = SimManGui
        RosT = SimManROS
        
        super(SimManRqt, self).__init__(context)
        self.setObjectName(name)

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print "arguments: ", args
            print "unknowns: ", unknowns

        # Create QWidget
        self._gui = GuiT()
        self._ros = RosT()

        loadUi(resource, self._gui)
        self._ros.setup(self._gui)
        self._gui.setup(name + "Rqt", self._ros)

        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + 
                                        (" (%d)" % context.serial_number()))
        context.add_widget(self._gui) 

if __name__ == '__main__':
    from labust_rqt import rqt_plugin_meta
    name = "SimMan"
    resource = rqt_plugin_meta.resource_rpath(name, __file__)
    rqt_plugin_meta.launch_standalone(name = name,
                    GuiT = SimManGui,
                    RosT = SimManROS,
                    resource = resource);     
