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
pi_bar = [[] for _ in range(h.config.auvNum)]
setRx = [[0] for _ in range(h.config.targetNum)]

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
    
    '''NB Ricorda che più è grande il pacchetto più tempo impieghi a trasmettere un messaggio.
    Ciò significa che quando decidi il Time Slot (Ts), devi essere sicuro che la dimensione del pacchetto
    sia proporzionata al tempo di trasmissione. Qualora vuoi l'AUV2 come ponte, verifica
    se riesci ad aggregare le policy of intent. Comunque il focus sarà sul fatto che
     l'algoritmo funziona con l'aggregazione minima possibile. Adesso in teoria sei nel setup più sfigato
    perchè 1 e 3 non sanno niente delle loro info recripoche, solo pos iniziale, neanche le misure. '''


    rospy.sleep(1)
    ## SIMULATION LOOP ############################################################################################################
    while not rospy.is_shutdown():

        # Retrieve current measurement and check for new data
        measRx = setRx[0]
        if measRx[0] - measRxOld[0] > 1:
            # Find the first empty buffer slot and update with new measurements
            try:
                idx = buffer.index([0])
                tmp_setRx = [[tmp[0], tmp[1], tmp[2], tmp[3], tmp[4]] for tmp in setRx]
                buffer[idx] = tmp_setRx
            except ValueError:
                rospy.logwarn("No empty buffer slot available")

        # Process each measurement in the buffer up to the current index
        for i in range(idx):
            setTx = buffer[i]
            measure = setTx[-1]
            delay[i] += dt

            if measure != 0:
                # Calculate distance only once for this measure
                d = np.sqrt((auv_xy[1] - measure[3]) ** 2 + (auv_xy[0] - measure[2]) ** 2)
                transmission_time = d * (1 / c) * 2  # Calculate once

                # Check if delay has met the required transmission time
                if delay[i] >= transmission_time:
                    idx_rmv.append(i)
                    update_buff = True

                    # Simulate packet delivery
                    if h.simulatePktDelivery(PDR):
                        rcvd_pkt += 1
                        setTx_np = np.array(setTx, dtype=np.float32)
                        rows, cols = setTx_np.shape

                        # Publish data
                        pub_rx_meas.publish(Matrix(data=setTx_np.flatten().tolist(), rows=rows, cols=cols))
                        pi_i = pi_bar[auvID - 1]
                        pub_intent.publish(np.array(pi_i, dtype=np.float32))
                    else:
                        lost_pkt += 1
                        rospy.logwarn(f'|---- ACOUSTIC MODEM {auvID}: Lost a Packet')

        # Update the buffer if needed
        if update_buff:
            # Remove indexed elements in one go and append placeholders
            for i in sorted(idx_rmv, reverse=True):
                buffer.pop(i)
                delay.pop(i)
            buffer.extend([[0]] * len(idx_rmv))
            delay.extend([0] * len(idx_rmv))

            # Reset flags and indices
            idx_rmv.clear()
            update_buff = False
            idx = buffer.index([0])

        # End simulation if time limit is reached
        if int(t) == (h.config.TIME_DURATION - 1):
            rospy.on_shutdown(shutdown_cllbk)
            rospy.signal_shutdown('Simulation time limit reached')

        # Update old measurement, time, and counter, then sleep
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
    for i in range(auvNum):
        if auvID != i+1:
            if i+1 in netTopology:
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



