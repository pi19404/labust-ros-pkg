#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import sys
from python_qt_binding import QtCore, QtGui
from dot_graph import DotGraph

class HLManagerGui(QtGui.QWidget):
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)
        
        self._depGraph = DotGraph()
        self._pnGraph = DotGraph()
        
        self._dep_view = QtGui.QGraphicsView(self._depGraph)
        self._pn_view = QtGui.QGraphicsView(self._pnGraph)
                     
        pass

    def setup(self, name, ros):
        self.setObjectName(name);
        pass
    
    def unload(self):
        pass

    def drawDepGraph(self,data):
        self._depGraph.drawDot(data)
        self._dep_view.show();
        
    def drawPNGraph(self,data):
        self._pnGraph.drawDot(data)
        self._pn_view.show();


if __name__ == '__main__':
    import rospy
    import os
    from HLManager_ros import HLManagerROS
    import python_qt_binding as pyqt
    app = QtGui.QApplication(sys.argv)
    rospy.init_node("HLManager")
    gui = HLManagerGui()
    ros = HLManagerROS()
    
    #Get file path
    ui_file = os.path.join(
                os.path.dirname(
                  os.path.realpath(__file__)), 
                         'resource/HLManager.ui')
    #Load the UI
    pyqt.loadUi(ui_file, gui)   
    #Connect the user interface and ROS
    ros.setup(gui)
    gui.setup("HLManager", ros)
    #Start GUI
    gui.show()
    
    sys.exit(app.exec_())