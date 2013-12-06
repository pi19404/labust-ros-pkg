'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import os

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi, QtCore

class RqtPluginMeta(Plugin):
    def __init__(self, 
                 context, 
                 name,
                 GuiT, 
                 RosT, 
                 resource = ""
                 ):
        super(RqtPluginMeta, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName(name)
        
        if resource == "": resource=_resource(name, file)      
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
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

        # Extend the widget with all attributes and children from UI file
        loadUi(resource, self._gui)
        # Give QObjects reasonable names
        self._ros.setup(self._gui)
        self._gui.setup(name + "Rqt", self._ros)
        
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + 
                                        (" (%d)" % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._gui)      

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
        
def _resource(name):
    return os.path.join(os.path.dirname(
                        os.path.realpath(__file__)), 
                        "resource/" + name + ".ui")    
    
def resource_rpath(name, file):
    return os.path.join(os.path.dirname(
                        os.path.realpath(file)), 
                        "resource/" + name + ".ui")             
        
def launch_standalone(name,
                      GuiT, 
                      RosT, 
                      resource = ""):
    import rospy, os, sys
    import python_qt_binding as pyqt
    from python_qt_binding import QtGui
    app = QtGui.QApplication(sys.argv)
    rospy.init_node(name)
    gui = GuiT()
    ros = RosT()
    
    if resource == "": 
        resource = _resource(name)
    #Load the UI
    pyqt.loadUi(resource, gui)   
    #Connect the user interface and ROS
    ros.setup(gui)
    gui.setup(name, ros)
    #Start GUI
    gui.show()
    
    sys.exit(app.exec_())  