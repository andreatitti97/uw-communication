#!/usr/bin/env python
#Import basic system modules
import os, importlib.util, pathlib
# Import numpy modules
import numpy as np
import random
#Import ROS modules
import rospy
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
from uwmsn_msgs.msg import Matrix

# Load the h file as a Python module 
pkg_directory = os.path.dirname(os.path.dirname(pathlib.Path(__file__).parent.resolve()))
header_file = pkg_directory+'/uw-communication'+'/include'+'/uw-communication'
log_path = pkg_directory+'/logs'

spec = importlib.util.spec_from_file_location("module.header", header_file+'/acoustic_modem_h.py')
h = importlib.util.module_from_spec(spec)
spec.loader.exec_module(h)


# VELOCITÀ DI COMUNICATION FISSA = 480 bps!
# Init Global Variables for callbacks
rcvd_pkt, lost_pkt = 0, 0
pi_bar = [[] for _ in range(len(h.config.AUV_XY))]
setRx = [[0],[0],[0],[0],[0]]

def run_acoustic_modem(pub_rx_meas,pub_intent ,auvID, auvNum):

    """Simulate the sensor platform and the moving target
    Input:  
            pub_rx_meas : object containing the publishers for received msgs
            pub_intent : object containing the publishers for policy of intent
            auvID : ID of the AUV associated with acoustic modem node
            auvNum : number ora AUVs
    """
    global count1, setRx, auv_xy, pi_bar, rcvd_pkt, lost_pkt

    # Rospy sim params
    Hz = 1/(h.config.TIME_STEP) #NB: different from sampling rate for move things, this is ros rate   
    rate = rospy.Rate(Hz)

    # Init variables, lists, bool
    t, count1, idx = 0,0,0
    delay, buffer, idx_rmv = [], [], []
    measRxOld = [0,0,0,0,0]#[t,meas,psx,psy,label]
    update_buff = False
    
    # Load simulation parameters from config file
    t_scaler = h.config.TIME_SCALER
    dt = h.config.TIME_STEP*t_scaler
    c = h.config.c #(m/s)
    PDR = h.config.PDR
    buffLen = 30
    for i in range(buffLen):
        delay.append(0)
        buffer.append([0])#create a buffer 
    
    rospy.sleep(1)
    ## SIMULATION LOOP ############################################################################################################
    while not rospy.is_shutdown():

        measRx = setRx[0]
        
        if measRx[0] - measRxOld[0] > 1:#check if a new set of meas has been overwritten
            
            # Check where to put the measurement in the buffer
            idx = buffer.index([0])#put the measurement at the index with
            #convert received numpy array to list
            tmp_setRx = []
            for i in range(len(setRx)):
                tmp = setRx[i]
                tmp_setRx.append([tmp[0],tmp[1],tmp[2],tmp[3],tmp[4]])

            buffer[idx] = tmp_setRx

        for i in range(len(buffer[0:idx])):
            
            setTx = buffer[i]
            measure = setTx[-1]#take the last measure (the one done just before transmitting) of the set as reference for computin SNR
            delay[i] += dt
            
            if measure != 0:
                d = np.sqrt((auv_xy[1]-measure[3])**2+(auv_xy[0]-measure[2])**2)

                if delay[i] >= (d*(1/c)*2) or d < 10:#latencies according to delay

                    idx_rmv.append(i)
                    update_buff = True                    
                    if h.simulatePktDelivery(PDR):
                        #msg received
                        rcvd_pkt += 1
                        setTx = np.array(setTx,dtype=np.float32)
                        rows, cols = setTx.shape
                        # First Traffic - consensus (distributed estimation is better)
                        pub_rx_meas.publish(Matrix(data=setTx.flatten().tolist(), rows=rows, cols=cols))
                        tmp1 = pi_bar[auvID-1]
                        # Second Traffic - Cooperation and Navigation (policy of intent)
                        pub_intent.publish(np.array(tmp1,dtype=np.float32))
                            
                    else:#msg lost
                        lost_pkt += 1
                        rospy.logwarn('|---- ACOUSTIC MODEM '+str(auvID)+': Lost a Packet')

        if update_buff == True:
            # Update the buffer according to the pkt sent
            for i in range(len(idx_rmv)-1):
                if len(idx_rmv) > 0:
                    buffer.pop(idx_rmv[i])
                    delay.pop(idx_rmv[i])
                    buffer.append([0])
                    delay.append(0)
                    idx_rmv = []
                    idx = buffer.index([0])
                    update_buff = False

        if int(t) == (h.config.TIME_DURATION-1):
            rospy.on_shutdown(shutdown_cllbk)
            rospy.signal_shutdown('Simulation time limit reached')

        measRxOld = measRx
        t += dt
        count1 += 1 
        rate.sleep()

def shutdown_cllbk():
    global auvID, lost_pkt, rcvd_pkt
    '''PUT DATA SAVING HERE'''
    if lost_pkt != 0 and rcvd_pkt != 0:
        PDR = rcvd_pkt*100/(lost_pkt+rcvd_pkt)
        np.savetxt(log_path+'/'+str(auvID)+'-PDR',[int(PDR)])

    magenta = "\033[0;35m"
    none = "\033[0m"
    rospy.loginfo('%s|---- ACOUSTIC MODEM '+str(auvID)+': Simulation data saved --> Shutting down ...%s',magenta,none)

def callbackAuvState(data):
    global auv_xy
    tmp = data.data
    auv_xy = [tmp[0],tmp[1]]

def callbackMeasTx(data):
    global setRx
    setRx = np.array(data.data).reshape(data.rows, data.cols)

def create_callback(index):
    def callback(data):
        global pi_bar
        pi_bar[index] = data.data
    return callback

def listener(auvID,auvNum,netTopology):

    """
    Sets up subscribers only for neighboring agents.
    
    Parameters:
    - auvID (int): ID of the current AUV (agent).
    - auvNum (int): Total number of AUVs.
    - netTopology (list): List of vectors defining network topology.
    """
    # Subscribe to each neighbor's topic
    print('AUV ID',auvID)
    print('network topologuy',netTopology)
    for i in range(auvNum):
        if auvID != i+1:
            if i+1 in netTopology:
                print('AUV ID',auvID)
                print('Number of Neighbours:',len(netTopology))
                
                rospy.Subscriber('/'+str(i+1)+'/tx_meas', Matrix, callbackMeasTx) 
        
        rospy.Subscriber('/'+str(i+1)+'/tx_ctrl_policy', numpy_msg(Floats), create_callback(i))
    rospy.Subscriber('vehicle_state_'+str(auvID), numpy_msg(Floats), callbackAuvState)

def main():

    # ROS INIT   
    namespace = rospy.get_namespace()
    params_path = namespace+'acoustic_modem'

    # Get AUV ID and number of vehicles and network topology
    global auvID, pi_bar
    auvID = rospy.get_param(params_path+'/auvID')
    auvNum = rospy.get_param(params_path+'/auvNum')
    netTopology = h.config.netTopology#should be added as ros param

    # Node Init
    rospy.init_node('acoustic_modem'+str(auvID)) #log_level=rospy.DEBUG

    # Publishers init
    pub_rx_meas = rospy.Publisher('/'+str(auvID)+'/rx_meas', Matrix, queue_size=100)
    pub_rx_ctrl_policy = rospy.Publisher('/'+str(auvID)+'/rx_ctrl_policy', numpy_msg(Floats), queue_size=1000)
    
    # Start simulation
    listener(auvID,auvNum,netTopology[auvID-1])
    run_acoustic_modem(pub_rx_meas, pub_rx_ctrl_policy, auvID,auvNum)
    rospy.on_shutdown(shutdown_cllbk)
    rospy.spin()

if __name__ == '__main__':
    main()



