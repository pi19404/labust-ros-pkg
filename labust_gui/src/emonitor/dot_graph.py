#!/usr/bin/env python
'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
from python_qt_binding import QtGui
from qt_dotgraph.dot_to_qt import DotToQtGenerator
from qt_dotgraph.pydotfactory import PydotFactory 
import pydot

class DotGraph(QtGui.QGraphicsScene):
    def __init__(self):
        #Init the base class
        QtGui.QGraphicsScene.__init__(self)
        
        # dot_to_qt transforms into Qt elements using dot layout
        self._dot_to_qt = DotToQtGenerator()
        #The pydotfactory
        self._dot_factory = PydotFactory()
        self._graph = None      
        pass
    
    def drawDot(self, data):        
        self._graph = pydot.dot_parser.parse_dot_data(data)
        
        #Clean current drawing
        self.clear()
        highlight_level = 3
 
        dot = self._dot_factory.create_dot(self._graph)
        # layout graph and create qt items
        (nodes, edges) = self._dot_to_qt.dotcode_to_qt_items(dot,
                                                            highlight_level,
                                                            True)
 
        for node_item in nodes.itervalues():
            self.addItem(node_item)
        for edge_items in edges.itervalues():
            for edge_item in edge_items:
                edge_item.add_to_scene(self)
 
        self.setSceneRect(self.itemsBoundingRect())