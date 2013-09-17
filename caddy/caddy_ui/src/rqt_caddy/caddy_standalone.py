#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import os
import sys
import rospy
import PyQt4 as python_qt_binding
from caddy_gui import CaddyGui
from caddy_plug import CaddyGuiROS
from PyQt4 import QtGui, uic

def loadUi(file, widget):
    return 

if __name__=="__main__":
    app = QtGui.QApplication(sys.argv)
    rospy.init_node("caddy_gui")
    gui = CaddyGui()
    ros = CaddyGuiROS(gui)
    ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'resource/CaddyGui.ui')
    # Extend the widget with all attributes and children from UI file
    gui._widget = uic.loadUi(ui_file)
    gui._widget.show();
    # Give QObjects reasonable names
    gui._widget.setObjectName('CaddyGui')
    gui.setup()
    ros.setup()
    
    sys.exit(app.exec_())
    