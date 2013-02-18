'''
Created on Feb 13, 2013

@author: dnad
'''
class Speed(object):
    '''
    The class generates speed trajectories based on configurations.
    '''
    def __init__(self, valueList):
        '''
        Constructor
        :param valueList: list of (time, value) pairs.
        '''
        self.valueList = valueList;
        
        self._timebase = 0;
        self._curVal = 0;
        self._curspeed = 0;
        
    def step(self, dT):
        from numpy import min
        
        if (self._timebase >= self.valueList[self._curVal][0]):
            self._curspeed = self.valueList[self._curVal][1];
            self._curVal=min([(self._curVal+1),len(self.valueList)-1]);
            
        self._timebase += dT;
            
        return self._curspeed;
            
            
        
        