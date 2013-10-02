'''
Created on Sep 10, 2013
 \todo Add license information here
@author: dnad
'''
import sys
#from PyQt4 import QtCore, QtGui, uic
from python_qt_binding import QtCore, QtGui, loadUi
from functools import partial

class CaddyGui(QtCore.QObject):
    def __init__(self):
        QtCore.QObject.__init__(self)
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
            n = text.find("\n");
            if n != -1:
                text = text[0:n] + "|||"+text[n:len(text)]; 
            chatHistory = self._widget.diverChatHistory;
    
        chatHistory.insertPlainText(text)

        
    def newDefaultMessage(self, idx):
        if idx in self.defaultMsgs.keys(): 
            self._widget.defaultTextEdit.insertHtml("<b>" + "Diver: " + "</b>");
            self._widget.defaultTextEdit.insertHtml(self.defaultMsgs[idx]);
            self._widget.defaultTextEdit.insertPlainText("\n");
        
    def newOriginPosition(self, data):
        self._widget.latLabel.setText(str(data.x))
        self._widget.lonLabel.setText(str(data.y))
        
    def newPladyposInfo(self,data):
        if len(data) >= 6:
            self._widget.p0field.setText(str(data[0]));
            self._widget.p1field.setText(str(data[1]));
            self._widget.p2field.setText(str(data[2]));
            self._widget.p3field.setText(str(data[3]));
            self._widget.currentField.setText(str(data[4]));
            self._widget.voltageField.setText(str(data[5]));
        
    def newManagerState(self,data):                      
        if data in self._managerButtons.keys():
            for key in self._managerButtons.keys():
                if key == data: 
                    self._managerButtons[data].setStyleSheet("color: green;");
                else:
                    self._managerButtons[key].setStyleSheet("");
                    
    def unload(self):
        from datetime import datetime;
        
        if self._widget.saveAll.isChecked():
            logFile = open(datetime.today().isoformat() + "_gui_logs.html",'w');
            for textEdit in (self._widget.diverChatHistory, 
                             self._widget.topsideChatHistory,
                             self._widget.defaultTextEdit):
                
                logFile.write(textEdit.toHtml());
                logFile.write("<br>");
            
            logFile.close(); 
        
    def _setup_diver_data(self):
        self.defaultMsgs = {0:"Alert",
                            1:"Ok",
                            2:"Yes",
                            3:"No",
                            4:"Repeat again",
                            5:"Diving out",
                            6:"Next kml set",
                            7:"End kml",
                            8:"InitReq"};              
        
    def _setup_gui(self):
        #Fill default combo box
        self._widget.defaultComboBox.addItem("Everything OK?");
        self._widget.defaultComboBox.addItem("Ok");
        self._widget.defaultComboBox.addItem("Yes");
        self._widget.defaultComboBox.addItem("No");
        self._widget.defaultComboBox.addItem("Repeat again");
        self._widget.defaultComboBox.addItem("Dive out");
        
        #Fill manager state indicators
        self._managerButtons={0: self._widget.idleButton,
                              1: self._widget.initButton,
                              2: self._widget.waitButton,
                              3: self._widget.transmitButton};
                              
        QtCore.QObject.connect(self, QtCore.SIGNAL("onSendKml"), self._exthook.sendKml)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onSendText"), self._exthook.sendText)
        QtCore.QObject.connect(self, QtCore.SIGNAL("onSendDefault"), self._exthook.sendDefault)
       # QtCore.QObject.connect(self, QtCore.SIGNAL("onManagerState"), self._gui.newManagerState)
        
        #Connect signals and slots
        self._widget.chatInput.returnPressed.connect(self._newTopsideMessage)
        self._widget.loadKml.clicked.connect(self._onLoadKml)
        self._widget.sendDefault.clicked.connect(self._onSendDefault);        
        self._widget.sendKml.clicked.connect(
                    lambda: 
                        self.emit(QtCore.SIGNAL("onSendKml"),self._widget.kmlFilePath.text()));      
        
        for key, button in self._managerButtons.iteritems():
            button.clicked.connect(partial(self._exthook.setManagerState,key))
    
    def _newTopsideMessage(self):
        text = (self._widget.chatInput.text() + "\n");
        self.newChatMessage(text)
        self.emit(QtCore.SIGNAL("onSendText"),text);
        #clear input   
        self._widget.chatInput.setText("");
        
    def _onSendDefault(self):
        self._widget.defaultTextEdit.insertHtml("<b>" + "Topside: " + "</b>");
        self._widget.defaultTextEdit.insertHtml(self._widget.defaultComboBox.currentText());
        self._widget.defaultTextEdit.insertPlainText("\n");
        self.emit(QtCore.SIGNAL("onSendDefault"),self._widget.defaultComboBox.currentIndex());
        
    def _onLoadKml(self):
        self._widget.kmlFilePath.setText(QtGui.QFileDialog.getOpenFileName(self._widget, "Get KML file",".","Google KML (*.kml)")[0]);
                
if __name__ == '__main__':
    app = QtGui.QApplication(sys.argv)
    window = CaddyGui()
    sys.exit(app.exec_())