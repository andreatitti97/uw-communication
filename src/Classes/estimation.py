#Import basic system modules
import os
import time
import importlib.util
import numpy as np
from numpy import append, matlib
# Import Costum classes
class_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)

def state_vector_to_scalars(state_vector):
    '''
    Returns the elements from the state_vector as a tuple of scalars.
    '''
    return (state_vector[0][0,0],state_vector[1][0,0],state_vector[2][0,0],state_vector[3][0,0])    
    
class Estimator:
    def __init__(self):
        '''
        Each object being tracked will result in the creation of a new ExtendedKalmanFilter instance.
        '''
        self.__x = []
        self.__phi = []
        self.__y = []
        self.__C = matlib.zeros((1,4))
        self.__t = []
        self.t_prev = 0

    @property
    def current_estimate(self):
        return (self.__x, self.__phi, self.__y) #self.__w

    def init_state_vector(self):
        return True

    def propagation(self, curr_time, prev_time):#compute old state in the regressor and propagation to the actual state
        # Propagate the estimation
        dt = curr_time - self.__t[0] #tempo attuale - tempo ultimo stato noto.
        self.__F = np.matrix([[1,0,dt,0],
                              [0,1,0,dt],
                              [0,0,1,0],
                              [0,0,0,1]])
        self.__x = self.__F*self.__x

    def iteration(self, t_meas, y_i, si_x, si_y, prev_t):
        
        self.__y.append(si_x*np.sin(y_i) - si_y*np.cos(y_i))
        self.__t.append(t_meas)
        prev_t = self.__t[0]
        self.__C = [np.sin(y_i), -np.cos(y_i), (t_meas - self.__t[0])*np.sin(y_i), -(t_meas - self.__t[0])*np.cos(y_i)]
        self.__phi.append(self.__C)

        if len(self.__y) == config.TP:
            self.__y.pop(0) #SHIFT
            self.__t.pop(0)
            self.__phi.pop(0)          
        for i in range(len(self.__phi)): # UPDATE REGRESSOR COLUMN
            if prev_t!=self.__t[i]:
                tmp = self.__phi[i]
                tmp[2] = ((self.__t[i]-self.__t[0])/(self.__t[i]-prev_t))*tmp[2]
                tmp[3] = ((self.__t[i]-self.__t[0])/(self.__t[i]-prev_t))*tmp[3]
                self.__phi[i] = [tmp[0],tmp[1],tmp[2],tmp[3]]
        # Invert the equations for computing the state at the oldest time in the regressor
        tmp_y = np.zeros((len(self.__y),1))
        for i in range(len(self.__y)):
            tmp_y[i] = self.__y[i]
        self.__x = np.dot(np.linalg.pinv(self.__phi),tmp_y)


        
        

        

