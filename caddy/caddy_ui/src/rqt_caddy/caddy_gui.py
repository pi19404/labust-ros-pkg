'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import sys
#from PyQt4 import QtCore, QtGui, uic
from python_qt_binding import QtCore, QtGui, loadUi

class CaddyGui():
    def __init__(self):
        # Set up the user interface from Designer.
        #self._widget = uic.loadUi("resource/CaddyGui.ui")
        #self._widget.show()
        self._widget = QtGui.QWidget()
    
    def setup(self):    
        self._setup_diver_data()
        self._setup_gui()
        
    def bindHook(self, data):
        self._exthook = data        
    
    def newChatMessage(self, text, source = "Topside"):
        chatHistory = QtGui.QTextEdit()
        if source == "Topside":
            chatHistory = self._widget.topsideChatHistory;
        else:
            chatHistory = self._widget.diverChatHistory;
            
        chatHistory.insertHtml("<b>"+source+": </b>")
        chatHistory.insertHtml(text)
        chatHistory.insertPlainText("\n")
        
    def newDefaultMessage(self, idx):
        self._widget.defaultReceivedLabel.setText(self.defaultMsgs[idx])
        
    def _setup_diver_data(self):
        self.defaultMsgs = {0:"Alert",
                            1:"Ok",
                            2:"Yes",
                            3:"No",
                            4:"Repeat again",
                            5:"Diving out"};              
        
    def _setup_gui(self):
        #Fill default combo box
        self._widget.defaultComboBox.addItem("Everything OK?");
        self._widget.defaultComboBox.addItem("Ok");
        self._widget.defaultComboBox.addItem("Yes");
        self._widget.defaultComboBox.addItem("No");
        self._widget.defaultComboBox.addItem("Repeat again");
        self._widget.defaultComboBox.addItem("Dive out");
        #Connect signals and slots
        self._widget.chatInput.returnPressed.connect(self._newTopsideMessage)
        self._widget.sendDefault.clicked.connect(self._onSendDefault)
        self._widget.loadKml.clicked.connect(self._onLoadKml)
        self._widget.sendKml.clicked.connect(self._onSendKml)
    
    def _newTopsideMessage(self):
        self.newChatMessage(text = self._widget.chatInput.text())
        self._exthook.sendText(self._widget.chatInput.text());
                #clear input   
        self._widget.chatInput.setText("");
        
    def _onSendDefault(self):
        print "Send default message with idx: " + str(self._widget.defaultComboBox.currentIndex());
        self._exthook.sendDefault(self._widget.defaultComboBox.currentIndex());
        #Add connection to ROS
        
    def _onLoadKml(self):
        self._widget.kmlFilePath.setText(QtGui.QFileDialog.getOpenFileName(self._widget, "Get KML file",".","Google KML (*.kml)")[0]);
                 
    def _onSendKml(self):
        #Add connection to ROS
        print "send: " + self._widget.kmlFilePath.text()
        self._exthook.sendKml(self._widget.kmlFilePath.text())
        
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = CaddyGui()
    sys.exit(app.exec_())