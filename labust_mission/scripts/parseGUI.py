#!/usr/bin/env python

import os, sys, subprocess, rospy
from PySide import QtCore, QtGui, QtUiTools

from std_msgs.msg import Bool
from std_msgs.msg import String
from misc_msgs.msg import StartParser 



class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent)
        self.loadUiWidget("/home/filip/catkin_ws/src/labust-ros-pkg/labust_mission/scripts/parsegui.ui")
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
        self.pubStartParse = rospy.Publisher('/startParse', StartParser)

        # Subscribers
        #rospy.Subscriber("/pladypos/stateHat", NavSts, self.callback)

        # Init node
        rospy.init_node('parseGUI')
        
        # Get parameters
        self.labustMissionPath = rospy.get_param('~labust_mission_path', '')


    def browseFiles(self):
        #fileName = QtGui.QFileDialog.getOpenFileName(self,
        #str("Open Image"), str("/home/filip"), str("Image Files (*.png *.jpg *.bmp)"))
        filename = QtGui.QFileDialog.getOpenFileName(self,
        str("Open Neptus mission file"), str("/home/"), str(""))
        print "Opened file: "+filename[0]
        self.filename = filename[0]
        self.ui.fileName.setText(str(filename[0]))
        bashCmd = str(self.labustMissionPath+"scripts/unzipMission.bash "+filename[0])
        subprocess.call(bashCmd,shell=True)

    def startParse(self):
        # Dodaj provjeru je li uspjelo unzipanje
            
        missionData = StartParser()
        missionData.fileName = self.labustMissionPath+"data/extracted/mission.nmis"
        
        if self.ui.radioButtonNED.isChecked():
            missionData.relative = True
        else:
            missionData.relative = False
        
        self.pubStartParse.publish(missionData)


if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = ControlMainWindow()
    MainWindow.ui.show()
    sys.exit(app.exec_())
