#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import rospy
from python_qt_binding import QtCore
from std_msgs.msg import String

class HLManagerROS(QtCore.QObject):
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
            
            
            