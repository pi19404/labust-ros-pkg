#!/usr/bin/env python

import os, sys, subprocess, rospy
from PySide import QtCore, QtGui, QtUiTools

from std_msgs.msg import Bool
from std_msgs.msg import String



class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.loadUiWidget("parsegui.ui")
        self.ui.browseButton.clicked.connect(self.browseFiles)
        self.ui.startButton.clicked.connect(self.startParse)
        self.initROS()


    def loadUiWidget(self,uifilename, parent=None):
        loader = QtUiTools.QUiLoader()
        uifile = QtCore.QFile(uifilename)
        uifile.open(QtCore.QFile.ReadOnly)
        self.ui = loader.load(uifile, parent)
        uifile.close()
        return 

    def initROS(self):
        # Publishers
        self.pubStartParse = rospy.Publisher('/startParse', String)

        # Subscribers
        #rospy.Subscriber("/pladypos/stateHat", NavSts, self.callback)

        # Init node
        rospy.init_node('parseGUI')

    def browseFiles(self):
        #fileName = QtGui.QFileDialog.getOpenFileName(self,
        #str("Open Image"), str("/home/filip"), str("Image Files (*.png *.jpg *.bmp)"))
        filename = QtGui.QFileDialog.getOpenFileName(self,
        str("Open Neptus mission file"), str("/home/filip"), str(""))
        print "Opened file: "+filename[0]
        self.filename = filename[0]
	self.ui.fileName.setText(str(filename[0]))
   
	bashCmd = str("/home/filip/catkin_ws/src/labust-ros-pkg/labust_mission/script/unzipMission.bash "+filename[0])
        #print bashCmd
        subprocess.call(bashCmd,shell=True)

    def startParse(self):
        # Dodaj provjeru je li uspjelo unzipanje
        missionFilename = String()
        missionFilename = self.filename
        self.pubStartParse.publish(missionFilename)


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = ControlMainWindow()
    MainWindow.ui.show()
    sys.exit(app.exec_())
