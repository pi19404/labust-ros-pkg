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

from python_qt_binding import loadUi, QtCore, QtGui
from qt_gui.plugin import Plugin

from labust_uvapp.srv import ConfigureVelocityController
from navcon_msgs.msg import DOFIdentificationAction, DOFIdentificationGoal
from navcon_msgs.msg import DOFIdentificationResult, DOFIdentificationFeedback
from navcon_msgs.msg import ModelParamsUpdate

from std_msgs.msg import String
from actionlib import SimpleActionClient, SimpleGoalState

class IdentSetup():
    def __init__(self):
        self.dof = 0
        self.ref = 0.0
        self.amp = 0.0
        self.hyst = 0.0
        self.Ts = 0.1
        self.yawCheck = False
        self.depthCheck = False
        self.positionCheck = False
            
class IdentificationGui(QtGui.QWidget):
    
    onStartIdent = QtCore.pyqtSignal(bool, object, name='onStartIdent')
    onTune = QtCore.pyqtSignal(object, bool, name='onTune')
    
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
        self.onTune.connect(ros.onModelUpdate)
        
        self.startIdent.clicked.connect(self._startIdent_pressed)
        self.stopIdent.clicked.connect(self._stopIdent_pressed)
        self.tuneButton.clicked.connect(self._tuneButton_pressed)
            
    @QtCore.pyqtSlot(bool)
    def onActionLib(self, data):
        self.useActionlib.setChecked(data)
        self.feedbackGroup.setEnabled(data)
        self.loadButton.setEnabled(not data)       
        
    @QtCore.pyqtSlot(object, object)
    def onActionResult(self, state, result):
        print("Finished with state:", state)
        print("Result:",result)
        self.resAlpha.setText("{:.3f}".format(result.alpha))
        self.resBeta.setText("{:.3f}".format(result.beta))
        self.resBetaa.setText("{:.3f}".format(result.betaa))
        self.resDelta.setText("{:.3f}".format(result.delta))
        self.resWn.setText("{:.3f}".format(result.wn))
               
        self.lastResult = result;  
        self.tuneButton.setEnabled(True)   
        self._ident_state(False)           
        
    @QtCore.pyqtSlot(object)
    def onActionFeed(self, feedback):
         print("Feedback:",feedback)
         self.fbDof.setText(str(feedback.dof))
         self.fbOsc.setText(str(feedback.oscillation_num))
         self.fbError.setText("{:.2f}".format(feedback.error * 100)+" %")
        
    @QtCore.pyqtSlot()
    def _startIdent_pressed(self):           
        tmp = IdentSetup()
        tmp.dof = self.dofCombo.currentIndex()
        tmp.ref = self.identRef.value()
        tmp.amp = self.identAmp.value()
        tmp.hyst = self.identHyst.value()
        tmp.yawCheck = self.yawCheck.isChecked()
        tmp.depthCheck = self.depthCheck.isChecked()
        tmp.positionCheck = self.positionCheck.isChecked()
        
        self._ident_state(True)
        self._clear_results()
        
        self.onStartIdent.emit(True, tmp)
        
    def _ident_state(self, onoff):
        self.startIdent.setEnabled(not onoff)
        self.stopIdent.setEnabled(onoff)
    
    def _clear_results(self):
        self.resAlpha.setText('0.00')
        self.resBeta.setText('0.00')
        self.resBetaa.setText('0.00')
        self.resDelta.setText('0.00')
        self.resWn.setText('0.00')       
    
    @QtCore.pyqtSlot()   
    def _stopIdent_pressed(self):
        self.onStartIdent.emit(False, IdentSetup())
        
    @QtCore.pyqtSlot()
    def _tuneButton_pressed(self):
        self.onTune.emit(self.lastResult, self.useLinear.isChecked())
                    
    def _setup(self):
        self.dofCombo.addItem("X")
        self.dofCombo.addItem("Y")
        self.dofCombo.addItem("Z")
        self.dofCombo.addItem("K")
        self.dofCombo.addItem("M")
        self.dofCombo.addItem("N")   
        
        self.canTune = False
        self.tuneButton.setEnabled(False)
        self.stopIdent.setEnabled(False)


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
        
        @QtCore.pyqtSlot(bool, object)
        def onIdentGuiReq(self, active, setup):
            if setup.dof < 0 or setup.dof >= 6:
                print("DOF select {} is invalid. Ignoring.".format(dof))
                return
                 
            if self.useAction:
                self._actionIdent(active, setup)
            else:
                self._velconIdent(active, setup)
        @QtCore.pyqtSlot(object, bool)
        def onModelUpdate(self, result, use_linear):
            update = ModelParamsUpdate()
            update.dof = result.dof
            update.alpha = result.alpha
            update.beta = result.beta
            update.betaa = result.betaa
            update.delta = result.delta
            update.wn = result.wn
            update.use_linear = use_linear
            self.model_update.publish(update)          
                
        def _velconIdent(self, active, setup):
            velcon = [0,0,0,0,0,0]
            '''Identification is number 3'''
            if active: 
                velcon[setup.dof] = 3
                print(setup.amp)
                rospy.set_param(self.velconName + "/" + 
                                self.names[setup.dof] +
                                "_ident_amplitude",setup.amp);
                rospy.set_param(self.velconName +"/" + 
                                self.names[setup.dof] + 
                                "_ident_hysteresis",setup.hyst);
                rospy.set_param(self.velconName + "/" + 
                                self.names[setup.dof] +
                                "_ident_ref",setup.ref);
                if setup.depthCheck: velcon[2] = 2;
                if setup.yawCheck: velcon[5] = 2;
                if setup.positionCheck: 
                    velcon[0] = 2;
                    velcon[1] = 2;
            
            try:
                configurer = rospy.ServiceProxy("ConfigureVelocityController", 
                                                    ConfigureVelocityController)
                configurer(desired_mode=velcon)
            except rospy.ServiceException, e:
                    print("Service call failed: %s"%e)
                    
        def _actionIdent(self, active, setup):
            #This is for using the velocity controller with external ident
            self._velconIdent(active,setup)
            if not active:
                self.ac.cancel_all_goals()
                return
            
            goal = DOFIdentificationGoal()
            goal.dof = setup.dof
            goal.command = setup.amp
            goal.reference = setup.ref
            goal.hysteresis = setup.hyst
            goal.sampling_rate = 1/setup.Ts
            
            #self._ac_onResult = lambda state, data: self._velconIdent(false,setup) self.onActionResult.emit(state,data)
            self._ac_onActive = lambda: print("Goal reported activation.")
            self._ac_onFeed = lambda data: self.onActionFeed.emit(data)            
            self.ac.send_goal(goal, self._ac_onResult, self._ac_onActive, self._ac_onFeed)
                        
        def _ac_onResult(self, state, data):
            self._velconIdent(False,None) 
            self.onActionResult.emit(state,data)      
        
        def unload(self):
            pass

        def _connect_signals(self, gui):
            self.onActionLib.connect(gui.onActionLib)
            self.onActionFeed.connect(gui.onActionFeed)
            self.onActionResult.connect(gui.onActionResult)
            
        def _subscribe(self):
            self.useAction = rospy.get_param("~use_action",True)
            self.velconName = rospy.get_param("~velcon_name","velcon")
            self.model_update = rospy.Publisher("model_update", ModelParamsUpdate)
            
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


class IdentificationRqt(Plugin):
    def __init__(self, context):
        name = "Identification"
        resource = os.path.join(os.path.dirname(
                        os.path.realpath(__file__)), 
                        "resource/" + name + ".ui")
        GuiT = IdentificationGui
        RosT = IdentificationROS
        
        super(IdentificationRqt, self).__init__(context)
        self.setObjectName(name)

        from argparse import ArgumentParser
        parser = ArgumentParser()
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print("arguments: ", args)
            print("unknowns: ", unknowns)

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
    name = "Identification"
    resource = rqt_plugin_meta.resource_rpath(name, __file__)
    rqt_plugin_meta.launch_standalone(name = name,
                    GuiT = IdentificationGui,
                    RosT = IdentificationROS,
                    resource = resource);     