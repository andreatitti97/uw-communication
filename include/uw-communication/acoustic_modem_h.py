#!/usr/bin/env python
import os
import importlib.util, pathlib
import random
import numpy as np

# Import Costum classes
class_path = pathlib.Path(__file__).parent.resolve()
class_path = os.path.dirname(os.path.dirname(os.path.dirname(class_path)))
class_path = class_path+'/uwmsn-sim'+'/src'+'/Classes'

spec = importlib.util.spec_from_file_location("module.config", class_path+'/config.py')
config = importlib.util.module_from_spec(spec)
spec.loader.exec_module(config)
spec = importlib.util.spec_from_file_location("module.sensor", class_path+'/sensor.py')
sensor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sensor)
spec = importlib.util.spec_from_file_location("module.tracker", class_path+'/tracker.py')
tracker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tracker)


def simulatePktDelivery(pdr, stddev=1):
    """
    Simulates packet transmission between two nodes based on packet delivery ratio.
    
    Parameters:
    - pdr (float): Mean packet delivery ratio as a percentage (0-100).
    - stddev (float): Standard deviation of the packet delivery ratio, default is 5%.
    
    Returns:
    - bool: True for a successful transmission, False for a failed transmission.
    """
    # Generate a packet delivery success probability using a Gaussian distribution
    success_probability = random.gauss(pdr, stddev)
    
    # Clamp the probability to be between 0 and 100
    success_probability = max(0, min(100, success_probability))
    
    # Generate a random value between 0 and 100 to decide success or failure
    return np.random.uniform(0, 100) < success_probability
