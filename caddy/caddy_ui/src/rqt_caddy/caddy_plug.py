'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import os
import rospy

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore
from caddy_gui import CaddyGui
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Point

class CaddyGuiROS(QtCore.QObject):
    
    def __init__(self, gui):
        QtCore.QObject.__init__(self)
        self._gui = gui;
        self._gui.bindHook(self)
        
    def setup(self):
        #Define event
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDiverText"), self._gui.newChatMessage)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDiverDefaults"), self._gui.newDefaultMessage)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onDiverOrigin"), self._gui.newOriginPosition)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onManagerState"), self._gui.newManagerState)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onPladyposInfo"), self._gui.newPladyposInfo)
        
        self.diverText = rospy.Subscriber("diver_text",String,
                            lambda data:
                                self.emit(QtCore.SIGNAL("onDiverText"),data.data, "Diver"));
        self.defaultMsgs = rospy.Subscriber("diver_defaults",Int32,
                            lambda data:
                                self.emit(QtCore.SIGNAL("onDiverDefaults"),data.data));
        self.diverOrigin = rospy.Subscriber("diver_origin",Point,
                            lambda data: 
                                self.emit(QtCore.SIGNAL("onDiverOrigin"),data));
        self.managerState = rospy.Subscriber("usbl_current_state",Int32,
                            lambda data:
                                self.emit(QtCore.SIGNAL("onManagerState"),data.data));
        self.pladyposInfo = rospy.Subscriber("pladypos_info",Float32MultiArray,
                            lambda data:
                                self.emit(QtCore.SIGNAL("onPladyposInfo"),data.data));
            
        self.outText = rospy.Publisher("usbl_text", String);
        self.outDefaults = rospy.Publisher("usbl_defaults",Int32);
        self.outKml = rospy.Publisher("kml_file",String);
        self.outMode = rospy.Publisher("usbl_force_state",Int32);
        
    def sendText(self, text):
        data = String();
        data.data = text;
        self.outText.publish(data);
    
    def sendDefault(self, idx):
        data = Int32();
        data.data = idx;
        self.outDefaults.publish(data);
        
    def sendKml(self, path):
        data = String();
        data.data = path;
        self.outKml.publish(data);
        
    def setManagerState(self, state):
        data = Int32();
        data.data = state;
        self.outMode.publish(data);
              
    def unload(self):
        self.diverText.unregister();
        self.defaultMsgs.unregister();
        self.managerState.unregister();
        self.diverOrigin.unregister();       
        self.outText.unregister();
        self.outDefaults.unregister();
        self.outKml.unregister();
        self.outMode.unregister();
        self.pladyposInfo.unregister();

class CaddyGuiPlug(Plugin):

    def __init__(self, context):
        super(CaddyGuiPlug, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CaddyGuiPlug')

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._gui = CaddyGui()
        self._ros = CaddyGuiROS(self._gui)
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource/CaddyGui.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._gui._widget)
        # Give QObjects reasonable names
        self._gui._widget.setObjectName('CaddyGui')
        self._gui.setup()
        self._ros.setup()
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._gui._widget)      

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        self._gui.unload();
        self._ros.unload()
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog