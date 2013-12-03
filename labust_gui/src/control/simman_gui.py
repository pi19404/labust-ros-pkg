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

from std_msgs.msg import String

class SimManGui(QtGui.QWidget):
    def __init__(self):
        #Init the base class
        QtGui.QWidget.__init__(self)
                    
        pass

    def setup(self, name, ros):
        self.setObjectName(name);
        pass
    
    def unload(self):
        pass

    def drawDepGraph(self,data):
        pass
        
    def drawPNGraph(self,data):
        pass


class SimManROS(QtCore.QObject):
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


# class SimManRqt(RqtPluginMeta):
#     def __init__(self, context):
#         super(SimManRqt, self).__init__(context = context,
#                                                 name = "SimMan",
#                                                 GuiT = SimManGui,
#                                                 RosT = SimManROS);

if __name__ == '__main__':
    from labust_rqt import rqt_plugin_meta
    rqt_plugin_meta.launch_standalone(name = "SimMan",
                    GuiT = SimManGui,
                    RosT = SimManROS);     