#!/usr/bin/env python
'''
Created on Feb 04, 2013

@author: dnad
'''
import rospy;
from threading import Lock
from collections import OrderedDict
import functools

#\todo Add this to useful tools module in python
#\todo Add imu message, GPS message logging, imu raw logging
def dataToList(data, logOrder): 
    return [reduce(getattr, elem.split("."), data)
            for elem in logOrder];
            
def dataToStrList(data, logOrder): 
    return [str(reduce(getattr, elem.split("."), data))
            for elem in logOrder];
            
class MessageLogger:
    def __init__(self, names, Type, logOrder, isList = False):
        #self.names = rospy.get_param(names);
        self.names = names;
        if isList:
             self.states = OrderedDict.fromkeys(self.names, [0.0 for elem in logOrder]);
        else:
            self.states = OrderedDict.fromkeys(self.names, Type());
            
        self.Type = Type;
        self.stateMux = Lock();
        self.logOrder=logOrder;
        self.isList = isList;
           
    def subscribe(self):
        for key in self.states.keys():
            rospy.Subscriber(key, self.Type,
                             functools.partial(self.onState,key));
    
    def onState(self,name,data):
        self.stateMux.acquire();
        if self.isList:
            self.states[name] = data.data;
        else:
            self.states[name] = data;
            
        self.stateMux.release();
        
    def getHeader(self):
        retval=[];
        for name in self.names:
            retval.append("%element: "+name+" \n");
            retval.extend(["% " + elem + "\n" for elem in self.logOrder]);

        return retval;
                
    def getLogLine(self):
        self.stateMux.acquire();
        if self.isList:
            retval = ["," + ",".join([str(elem) for elem in self.states[name]]) for name in self.names];
        else:
            retval = ["," + ",".join(dataToStrList(self.states[name], self.logOrder))  for name in self.names];
            
        self.stateMux.release();
        return retval;
     
class MatLogger:
    def __init__(self):
        logData = rospy.get_param("log_data");
        
        self.loggers=[]
        
        for dataType in logData:
            print ("Logging messages {}" 
                    "of type {}.").format(
                                          str(dataType[1]),
                                          str(dataType[0]));
            logOrder = rospy.get_param(str(dataType[0]));
            print ("Importing message type:"
                   "from {}.msg import {}").format(
                                             logOrder[0],
                                             logOrder[1]);
            module=__import__(name=logOrder[0]+".msg",
                                 fromlist=[logOrder[1]]);      
            print ("Adding message logger of type"
                 "{} with log order {}.").format(
                                                 getattr(module, logOrder[1]),
                                                 logOrder[2])
            isList = (len(logOrder) == 4) and logOrder[3] == 'list';
            self.loggers.append(MessageLogger(dataType[1], 
                                getattr(module, logOrder[1]), 
                                logOrder[2], isList))
                             
        from datetime import datetime;
        name = rospy.get_param("~filename","log");
        dir = rospy.get_param("~dir",".");             
        self.logFile = open(dir + "/" + name + "_" + datetime.today().isoformat() + ".csv",'w');
                            
        for logger in self.loggers: 
            logger.subscribe();
               
        self.writeHeader();

    def writeHeader(self):
        self.logFile.write("%The following element order can be found:\n");
        self.logFile.write("%element: stamp \n% t \n");
        for logger in self.loggers:
            self.logFile.writelines(logger.getHeader());                          
                           
    def start(self):
        rate = rospy.Rate(10);
        
        while not rospy.is_shutdown():
            self.logFile.write(str(rospy.Time.now().to_sec()));     
            for logger in self.loggers:
                self.logFile.writelines(logger.getLogLine());
            
            self.logFile.write("\n");            
            rate.sleep();
            
    def stop(self):
        self.logFile.close();
        
                           
if __name__ == "__main__":
    rospy.init_node("logger");
    log = MatLogger(); 
    log.start();
    while not rospy.is_shutdown():
        rospy.spin();
    log.stop();
        
        
        
