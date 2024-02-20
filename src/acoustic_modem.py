#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util, pathlib

# Import math modules
from math import pi, atan2
import numpy as np
from scipy import stats

#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Import Costum classes
class_path = pathlib.Path(__file__).parent.resolve()
class_path = class_path/'Classes'
spec = importlib.util.spec_from_file_location("module.config", class_path/'config.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.sensor", class_path/'sensor.py')
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
spec = importlib.util.spec_from_file_location("module.tracker", class_path/'tracker.py')
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)

# Log path definition
plot_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/logs/plot')

# Initi Global Variables 
m_rx = [0,0,0,0]
m_tx_local = [0,0,0,0]

def sig(x,d):
    
    alpha = -0.003
    gamma = d
    return 1/(1+np.e**(alpha*(gamma-x)))

def run_simulation():

    """Simulate the sensor platform and the moving target
    Input:  target : target initial state
            obs : list containing already initialized classes Tracker() (reproduce the local estimations)
            auv : list containing sensors state and methods for measurements
            pub : list containing the publishers
            cpf_control : already initialized class for CPF
            f : choosen geometry
            meas : initial s state
    """
    global count1, m_tx_local, meas_rx, t_pose

    Hz = 1/(config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate
    Hz = 1
    rate = rospy.Rate(Hz)

    # Init time variables and counters and lists
    t, count1, rcvd_pkt, lost_pkt = 0,0,0,0
    delay = []
    c = 1500 #(m/s)
    dt = config.TIME_STEP*config.TIME_SCALER
    meas_table = []
    old_m = [0,0,0,0]
    epsi = 0.01
    time.sleep(5000)
    ## SIMULATION LOOP ############################################################################################################
    while not rospy.is_shutdown():



        #TODO check if the NOT LOCAL measurements has alredy been processed.
        if sum(np.abs(m_rx[0:3]))-sum(np.abs(old_m[0:3]))>epsi:
            m_rx.append(m_tx_local[2])
            m_rx.append(m_tx_local[3])
            meas_table.append(m_rx)

            for i in range(len(meas_table)):
                delay[i] += dt
                d = np.sqrt((meas_table[i,5]-meas_table[i,3])**2+(meas_table[4]-meas_table[i,2])**2)
                prob = sig(d,d)
                if delay[i] >= d*(1/c):
                    if (np.random.random() <= prob):
                        #msg received
                        rcvd_pkt +=1
                        #if i+1 != auvID: #publish it
                        #    pub_rx_meas[i].publish(m_rx)
                            
                    else:#msg lost
                        lost_pkt += 1
                        print('LOST PACKETT')
                    
                    # REMOVE THE MEASUREMENT FROM THE TABLE 
                    meas_table.pop(i)
                    delay[i] = 0
        
        old_m = m_rx

        t += dt
        count1 += 1 

        rate.sleep()

def callback(data):
    
    global m_tx_local
    m_tx_local = data.data

def callback2(data):
    global m_rx
    
    m_rx = data.data

    
def listener(auvID,auvNum):

    #rospy.Subscriber('tx_meas_local', numpy_msg(Floats), callback)
    for i in range(auvNum):
        #if i+1 != auvID:
        rospy.Subscriber('/'+str(i+1)+'/tx_meas', numpy_msg(Floats), callback2)  
    
def main():

    # ROS INIT   
    namespace = rospy.get_namespace()
    params_path = namespace+'acoustic_modem'
    # Get AUV ID and number of vehicles.
    auvID = rospy.get_param(params_path+'/auvID')
    auvNum = rospy.get_param(params_path+'/auvNum')
    # Node Init
    rospy.init_node('acoustic_modem'+str(auvID))
    # Publishers init
    
    pub_rx_meas = []

    '''for i in range(auvNum):
        if i+1 != auvID:
            tmp = rospy.Publisher('/'+str(i+1)+'/uw_channel', numpy_msg(Floats), queue_size=100)
            pub_rx_meas.append(tmp)'''
    for i in range(auvNum):
        pub_rx_meas = rospy.Publisher('/'+str(i+1)+'/rx_meas', numpy_msg(Floats), queue_size=100)

    # Start listener and simulation
    listener(auvID,auvNum)
    run_simulation()

    rospy.spin()

if __name__ == '__main__':
    main()



