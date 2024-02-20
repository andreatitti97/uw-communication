from math import atan2, pi
import numpy as np

class Sensor:
    """ Simulate Vector Sensor """
    
    def __init__(self, name, f, mean, variance):
        self.name = name
        self.f = f
        self.mean = mean
        self.variance = variance*0.02
        self.abs_bearing = 0

    def measureBearing(self,xt,yt,obs_pos, orientation):


        vect = [xt-obs_pos[0],yt-obs_pos[1]]
        self.abs_bearing = atan2(vect[1],vect[0]) # abs bearing = rel_bearing - vehcile ori -> [-pi,+pi]

        if orientation < 0:
            theta_tmp = 2*pi + orientation
        else:
            theta_tmp = orientation

        if self.abs_bearing < 0:

            self.abs_bearing = 2*pi + self.abs_bearing # change convention Bearing_abs -> [0,2*pi]
        
        if theta_tmp  <= self.abs_bearing: # if the target is counter clock wise w.r.t to surge vel
            rel_bearing = self.abs_bearing - theta_tmp 
        else:
            rel_bearing = 2*pi - (theta_tmp - self.abs_bearing)

        # Create the noise and add the noise to the measurament
        self.noise = np.random.uniform(-self.variance, self.variance)
        self.abs_bearing = self.abs_bearing + self.noise #overwrite absolute bearing with the corrupted quantities
        return self.abs_bearing, rel_bearing, obs_pos

    