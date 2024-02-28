#!/usr/bin/env python
#Import basic system modules
import os
import time
import importlib.util, pathlib

# Import numpy modules
import numpy as np

#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

# Load the header file as a Python module 
pkg_directory = os.path.dirname(os.path.dirname(pathlib.Path(__file__).parent.resolve()))
header_file = pkg_directory+'/uw-communication'+'/include'+'/uw-communication'
log_path = pkg_directory+'/uwmsn-sim'+'/logs'

spec = importlib.util.spec_from_file_location("module.header", header_file+'/acoustic_modem_h.py')
header = importlib.util.module_from_spec(spec)
spec.loader.exec_module(header)

# Init Global Variables for callbacks
m_rx = [0,0,0,0]
rcvd_pkt, lost_pkt = 0, 0

def sig(x):
    
    alpha = header.config.alpha
    gamma = header.config.gamma
    return 1/(1+np.e**(alpha*(gamma-x)))

def run_acoustic_modem(pub_rx_meas,auvID,auvNum):

    """Simulate the sensor platform and the moving target
    Input:  
            pub_rx_meas : object containing the publisher for received msgs
            auvID : ID of the AUV associated with acoustic modem node
            auvNum : number ora AUVs
    """
    global count1, m_rx, auv_xy, rcvd_pkt, lost_pkt

    # ROS simulation parameters
    t_scaler = header.config.TIME_SCALER

    Hz = 1/(header.config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate   
    rate = rospy.Rate(Hz)

    # Init time variables and counters and lists
    t, count1, idx = 0,0,0
    dt = header.config.TIME_STEP*t_scaler
    delay, meas_table, idx_rmv = [], [], []
    c = header.config.c #(m/s)
    old_m = [0,0,0,0]
    buffLen = header.config.buffLen
    update_buff = False

    for i in range(buffLen):
        delay.append(0)
        meas_table.append(0)

    # Start listener
    listener(auvID,auvNum)

    rospy.sleep(1)
    ## SIMULATION LOOP ############################################################################################################
    while not rospy.is_shutdown():

        #TODO check if the measurements has alredy been processed.
        if m_rx[0] - old_m[0] > 1:#check if a new meas has been overwritten

            m_rx.append(auv_xy[0])
            m_rx.append(auv_xy[1])
            
            # Check where to put the measurement in the buffer
            idx = meas_table.index(0)
            meas_table[idx] = m_rx
                      
        for i in range(len(meas_table[0:idx])):
            
            measure = meas_table[i]         
            delay[i] += dt
            d = np.sqrt((measure[5]-measure[3])**2+(measure[4]-measure[2])**2)
            prob = sig(d)

            if delay[i] >= d*(1/c)*2 or d < 10:

                idx_rmv.append(i)
                update_buff = True

                if (np.random.random() <= prob) or d < 10:
                    #msg received
                    rcvd_pkt += 1
                    pub_rx_meas.publish(np.array(measure,dtype=np.float32))
                        
                else:#msg lost
                    lost_pkt += 1
                    rospy.logwarn('|---- ACOUSTIC MODEM '+str(auvID)+': Lost a Packet')

        if update_buff == True:
            # Update the buffer according to the pkt sent
            for i in range(len(idx_rmv)):
                meas_table.pop(idx_rmv[i])
                delay.pop(idx_rmv[i])
                meas_table.append(0)
                delay.append(0)
                idx_rmv = []
                idx = meas_table.index(0)
                update_buff = False

        if int(t) == (header.config.TIME_DURATION-1):
            rospy.on_shutdown(shutdown_cllbk)
            rospy.signal_shutdown('Simulation time limit reached')

        old_m = m_rx
        t += dt
        count1 += 1 
        rate.sleep()

def callback2(data):
    global m_rx
    
    tmp = data.data
    m_rx = [tmp[0],tmp[1],tmp[2],tmp[3]]

def shutdown_cllbk():
    global auvID, lost_pkt, rcvd_pkt
    '''PUT DATA SAVING HERE'''
    if lost_pkt != 0 and rcvd_pkt != 0:
        PDR = lost_pkt*100/(lost_pkt+rcvd_pkt)
    
        np.savetxt(log_path+'/'+str(auvID)+'-PDR',[PDR])
    magenta = "\033[0;35m"
    none = "\033[0m"
    rospy.loginfo('%s|---- ACOUSTIC MODEM '+str(auvID)+': Simulation data saved --> Shutting down ...%s',magenta,none)

def callbackAuvState(data):
    global auv_xy
    tmp = data.data
    auv_xy = [tmp[0],tmp[1]]

def listener(auvID,auvNum):

    rospy.Subscriber('vehicle_state_'+str(auvID), numpy_msg(Floats), callbackAuvState)
    for i in range(auvNum):
        rospy.Subscriber('/'+str(i+1)+'/tx_meas', numpy_msg(Floats), callback2)  
    
def main():

    # ROS INIT   
    namespace = rospy.get_namespace()
    params_path = namespace+'acoustic_modem'

    # Get AUV ID and number of vehicles.
    global auvID
    auvID = rospy.get_param(params_path+'/auvID')
    auvNum = rospy.get_param(params_path+'/auvNum')
 
    # Node Init
    rospy.init_node('acoustic_modem'+str(auvID)) #log_level=rospy.DEBUG

    # Publishers init
    pub_rx_meas = []
    for i in range(auvNum):
        tmp = rospy.Publisher('/'+str(i+1)+'/rx_meas', numpy_msg(Floats), queue_size=100)
        pub_rx_meas.append(tmp)
    pub_rx_meas = rospy.Publisher('/'+str(auvID)+'/rx_meas', numpy_msg(Floats), queue_size=100)

    # Start simulation
    run_acoustic_modem(pub_rx_meas,auvID,auvNum)
    rospy.on_shutdown(shutdown_cllbk)
    rospy.spin()

if __name__ == '__main__':
    main()



