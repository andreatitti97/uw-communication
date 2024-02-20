import os 
import importlib
import numpy as np
# Import costum modules
class_path = os.path.abspath('/home/andrea/Desktop/ros_simulation_ws/src/ipp_pkg/src/Classes')
spec = importlib.util.spec_from_file_location("module.config", class_path+"/config.py")
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.utils", class_path+"/utils.py")
utils = importlib.util.module_from_spec(spec)
spec.loader.exec_module(utils)

def saturateVel(linear_velocity):
    if config.MIN_TARGET_VEL < linear_velocity < config.MIN_TARGET_VEL:
        linear_velocity = config.MIN_TARGET_VEL
    if linear_velocity >= config.MAX_TARGET_VEL or linear_velocity <= -config.MAX_TARGET_VEL:
        linear_velocity = config.MAX_TARGET_VEL
    return np.abs(linear_velocity)

class Target:
    """
    Constructs an instantiate the AUV

    Parameters
    ----------
    max_linear_speed : (float)
        The maximum linear speed that the robot can go
    max_angular_speed : (float)
        The maximum angular speed that the robot can rotate about its vertical
        axis
    controller : (Controller)
        A configurable controller to finds the path and calculates command
        linear and angular velocities. 
    """

    def __init__(self): #path_finder_controller_target

        #self.target_controller = path_finder_controller_target # FOR FOLLOWING A POLYNOMIAL TRAJECTORY
        self.pose = utils.Pose(config.TARGET_INIT[0], config.TARGET_INIT[1],  config.TARGET_INIT[2])
        self.lin_vel = config.alpha_0 #(m/s)
        self.ang_vel = config.omega_0 #(rad/sec)
        self.lin_acc = config.alpha_dot_0 #(m/sec^2)
        self.ang_acc = config.omega_dot_0 #(rad/sec^2)

    def set_start_target_poses(self, pose):
        """
        Sets the start and target positions of the robot

        Parameters
        ----------
        pose_start : (Pose)
            Start postion of the robot (see the Pose class)
        pose : (Pose)
            Target postion of the robot (see the Pose class)
        """
        self.pose = pose


    def move_target(self, dt):
        """
        Moves the target for one time step increment

        Parameters
        ----------
        dt : (float)
            time step
        """
    
        self.pose.theta = self.pose.theta + (self.ang_vel+self.ang_acc * dt)*dt
        self.pose.x = self.pose.x + (self.lin_vel+self.lin_acc*dt) * \
            np.cos(self.pose.theta) * dt 
        self.pose.y = self.pose.y + (self.lin_vel+self.lin_acc*dt) * \
            np.sin(self.pose.theta) * dt
        
