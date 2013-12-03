#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import os, sys

import rospy
import labust_gui

from python_qt_binding import QtCore, QtGui
from labust_rqt.rqt_plugin_meta import RqtPluginMeta
from emonitor.dot_graph import DotGraph

from std_msgs.msg import String

class IdentificationGui(QtGui.QWidget):
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


class IdentificationROS(QtCore.QObject):
        def __init__(self):
            QtCore.QObject.__init__(self)
        
        def setup(self, gui):
            self._connect_signals(gui)
            self._subscribe()
            pass
        
        def unload(self):
            pass

        def _connect_signals(self, gui):
            QtCore.QObject.connect(self, 
                QtCore.SIGNAL("onDepGraph"), gui.drawDepGraph)
            QtCore.QObject.connect(self, 
                QtCore.SIGNAL("onPNGraph"), gui.drawPNGraph)
            
        def _subscribe(self):
            
            self._depGraph = rospy.Subscriber("dependency_graph", String,
                                lambda data:
                                    self.emit(QtCore.SIGNAL("onDepGraph"), 
                                          data.data))
            self._pnGraph = rospy.Subscriber("petri_net_graph", String,
                                lambda data:
                                    self.emit(QtCore.SIGNAL("onPNGraph"), 
                                          data.data))
            pass
        
        def _unsubscribe(self):
            self._depGraph.unregister()
            self._pnGraph.unregister()


# class IdentificationRqt(RqtPluginMeta):
#     def __init__(self, context):
#         super(IdentificationRqt, self).__init__(context = context,
#                                                 name = "Identification",
#                                                 GuiT = IdentificationGui,
#                                                 RosT = IdentificationROS);

if __name__ == '__main__':
    from labust_rqt import rqt_plugin_meta
    rqt_plugin_meta.launch_standalone(name = "Identification",
                    GuiT = IdentificationGui,
                    RosT = IdentificationROS);     