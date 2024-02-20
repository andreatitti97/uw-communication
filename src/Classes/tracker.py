import os
import importlib.util
import time
spec = importlib.util.spec_from_file_location("module.est", "/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/Classes/estimation.py")
est = importlib.util.module_from_spec(spec)
spec.loader.exec_module(est)
lib_path = os.path.abspath('/home/andrea/ros_simulation_ws/src/scripts/logs')

class Tracker:
    '''
    The Tracker class is created everytime we detect a target.
    It contains the entire state of the tracked object.
    '''
    def __init__(self):

        self.__estimator = est.Estimator()
        self.__curr_time = 0
        self.__prev_time = 0

    @property
    def state(self):
        return self.__estimator.current_estimate

    def processMeasurement(self, table): #table = [tempo, misura, auv pos x, auv pos y]
        for i in range(len(table)):
            input_data = table[i]
            self.__estimator.iteration(input_data[0], input_data[1], input_data[2], input_data[3], self.__prev_time)

    def propagate_estimation(self, curr_time):
        self.__curr_time = curr_time
        self.__estimator.propagation(self.__curr_time, self.__prev_time)
        self.__prev_time = self.__curr_time


        
