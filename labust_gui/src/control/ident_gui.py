#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
from __future__ import print_function
import os, sys

import rospy
import labust_gui

from python_qt_binding import QtCore, QtGui
from labust_rqt.rqt_plugin_meta import RqtPluginMeta

from labust_uvapp.srv import ConfigureVelocityController
from navcon_msgs.msg import DOFIdentificationAction, DOFIdentificationGoal
from navcon_msgs.msg import DOFIdentificationResult, DOFIdentificationFeedback

from std_msgs.msg import String
from actionlib import SimpleActionClient, SimpleGoalState

class IdentSetup:
    dof = 0
    ref = 0.0
    amp = 0.0
    hyst = 0.0
    Ts = 0.1     
    
class IdentificationGui(QtGui.QWidget):
    
    onStartIdent = QtCore.pyqtSignal(bool, IdentSetup.__type__, name='onStartIdent')
    
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)

    def setup(self, name, ros):
        self.setObjectName(name)
        self.setWindowTitle(name)
        self._setup();
        self._connect_signals(ros)   
    
    def unload(self):
        pass
    
    def _connect_signals(self, ros):
        self.onStartIdent.connect(ros.onIdentGuiReq)
        
        self.startIdent.clicked.connect(self._startIdent_pressed)
        self.stopIdent.clicked.connect(self._stopIdent_pressed)
            
    @QtCore.pyqtSlot(bool)
    def onActionLib(self, data):
        self.useActionlib.setChecked(data)
        self.feedbackGroup.setEnabled(data)
        self.loadButton.setEnabled(not data)       
        
    @QtCore.pyqtSlot(object, object)
    def onActionResult(self, state, resul):
        print("Finished with state:", state)
        result = DOFIdentificationResult()
        self.resAlpha = result.alpha
        self.resBeta = result.beta
        self.resBetaa = result.betaa
        self.resDelta = result.delta
        self.resWn = result.wn        
        
    @QtCore.pyqtSlot(object)
    def onActionFeed(self, feedback):
         self.fbDof.setText(str(feedback.dof))
         self.fbOsc.setText(str(feedback.oscillation_num))
         self.fbError.setText(str(feedback.error * 100)+" %")
        
    @QtCore.pyqtSlot()
    def _startIdent_pressed(self):
        if self.useActionlib.isChecked():
            self.canTune = False
            self.tuneButton.setEnabled(self.canTune)
            
        tmp = IdentSetup()
        tmp.dof = self.dofCombo.currentIndex()
        tmp.ref = self.identRef.value()
        tmp.amp = self.identAmp.value()
        tmp.hyst = self.identHyst.value()
        self.onStartIdent.emit(True, tmp)
    
    @QtCore.pyqtSlot()   
    def _stopIdent_pressed(self):
        self.onStartIdent.emit(False, IdentSetup())
          
    
    def _setup(self):
        self.dofCombo.addItem("X")
        self.dofCombo.addItem("Y")
        self.dofCombo.addItem("Z")
        self.dofCombo.addItem("K")
        self.dofCombo.addItem("M")
        self.dofCombo.addItem("N")   
        
        self.canTune = False
        self.tuneButton.setEnabled(self.canTune)  


class IdentificationROS(QtCore.QObject):
        onActionLib = QtCore.pyqtSignal(bool, name = "onActionLib")
        onActionResult = QtCore.pyqtSignal(object, object, name="onActionResult")
        onActionFeed = QtCore.pyqtSignal(object, name="onActionFeed")
        
        def __init__(self):
            QtCore.QObject.__init__(self)
            
            self.names=("Surge", 
                        "Sway",
                        "Heave",
                        "Roll",
                        "Pitch",
                        "Yaw")
        
        def setup(self, gui):
            self._connect_signals(gui)
            self._subscribe()
            pass
        
        @QtCore.pyqtSlot(bool, IdentSetup.__type__)
        def onIdentGuiReq(self, active, setup):
            if setup.dof < 0 or setup.dof >= 6:
                print("DOF select {} is invalid. Ignoring.".format(dof))
                return
                 
            if self.useAction:
                self._actionIdent(active, setup)
            else:
                self._velconIdent(active, setup)

                
        def _velconIdent(self, active, setup):
            velcon = [0,0,0,0,0,0]
            '''Identification is number 3'''
            if active: 
                velcon[setup.dof] = 3
                rospy.set_param(self.velconName + "/" + 
                                self.names[setup.dof] +
                                "_ident_amplitude",setup.amp);
                rospy.set_param(self.velconName +"/" + 
                                self.names[setup.dof] + 
                                "_ident_hysteresis",setup.hyst);
                rospy.set_param(self.velconName + "/" + 
                                self.names[setup.dof] +
                                "_ident_ref",setup.ref);
            
            try:
                configurer = rospy.ServiceProxy("ConfigureVelocityController", 
                                                    ConfigureVelocityController)
                configurer(desired_mode=velcon)
            except rospy.ServiceException, e:
                    print("Service call failed: %s"%e)
                    
        def _actionIdent(self, active, setup):
            if not active:
                self.ac.cancelAllGoals()
                return
            
            goal = DOFIdentificationGoal()
            goal.dof = setup.dof
            goal.command = setup.amp
            goal.reference = setup.ref
            goal.sampling_rate = setup.Ts
            
            self._ac_onResult = lambda state, data: self.onActionResult.emit(state,data)
            self._ac_onActive = lambda: print("Goal reported activation.")
            self._ac_onFeed = lambda data: self.onActionFeed.emit(data)            
            self.ac.sendGoal(goal, self._ac_onResult, self._ac_onActive, self._ac_onFeed)
            
            pass
        
        def unload(self):
            pass

        def _connect_signals(self, gui):
            self.onActionLib.connect(gui.onActionLib)
            self.onActionFeed.connect(gui.onActionFeed)
            self.onActionResult.connect(gui.onActionResult)
            
        def _subscribe(self):
            self.useAction = rospy.get_param("~use_action",False)
            self.velconName = rospy.get_param("~velcon_name","velcon")
            
            if self.useAction:
                '''Configure action server'''
                self.ac = SimpleActionClient("Identification", 
                                             DOFIdentificationAction)
                
                if not self.ac.wait_for_server(rospy.Duration(30.0)):
                    print("Action not supported, using fallback.")
                    self.useAction = False        
            
            self.onActionLib.emit(self.useAction) 
                                            
        def _unsubscribe(self):
            pass

class IdentificationRqt(RqtPluginMeta):
    def __init__(self, context):
        name = "Identification"
        resource = rqt_plugin_meta.resource_rpath(name, __file__)
        super(IdentificationRqt, self).__init__(context = context,
                                                name = name,
                                                GuiT = IdentificationGui,
                                                RosT = IdentificationROS,
                                                resource=resource)

if __name__ == '__main__':
    from labust_rqt import rqt_plugin_meta
    name = "Identification"
    resource = rqt_plugin_meta.resource_rpath(name, __file__)
    rqt_plugin_meta.launch_standalone(name = name,
                    GuiT = IdentificationGui,
                    RosT = IdentificationROS,
                    resource = resource);     