#!/usr/bin/env python

import os, sys, subprocess, rospy
from PySide import QtCore, QtGui, QtUiTools

from std_msgs.msg import Bool
from std_msgs.msg import String
from auv_msgs.msg import NavSts
from misc_msgs.msg import StartParser 


class ControlMainWindow(QtGui.QMainWindow):
    def __init__(self, parent=None):
        super(ControlMainWindow, self).__init__(parent) 

        self.loadUiWidget(os.path.join(
                        os.path.dirname(os.path.realpath(__file__)), "parsegui.ui"))
        
        self.ui.browseButton.clicked.connect(self.browseFiles)
        self.ui.startButton.clicked.connect(self.startParse)
        self.ui.stopButton.clicked.connect(self.stopMission)

        self.firstPass = True 
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
        self.pubStopMission = rospy.Publisher('/eventString', String)


        # Subscribers
        rospy.Subscriber("stateHat", NavSts, self.onStateHatCallback)

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
            missionData.customStart = False
        elif self.ui.radioButtonLatLon.isChecked():
            missionData.relative = False
            missionData.customStart = False
            missionData.lat = self.startLat
            missionData.lon = self.startLon
        else:
            missionData.relative = False
            missionData.customStart = True           
            missionData.lat = self.startLat
            missionData.lon = self.startLon
    
        self.pubStartParse.publish(missionData)
        
    def stopMission(self):
        data = String()
        data = "/STOP";
        self.pubStopMission.publish(data)
        
        
    def onStateHatCallback(self, msg):
        
        if self.firstPass:
            if msg.origin.latitude != 0 and msg.origin.longitude != 0:
                self.firstPass = False
            else:
                print "Waiting for origin"
            self.startLat = msg.origin.latitude
            self.startLon = msg.origin.longitude
            
            self.ui.lineEditLat.setText(str(self.startLat))
            self.ui.lineEditLon.setText(str(self.startLon))
        
if __name__ == "__main__":
    import sys
    app = QtGui.QApplication(sys.argv)
    MainWindow = ControlMainWindow()
    MainWindow.ui.show()
    sys.exit(app.exec_())
