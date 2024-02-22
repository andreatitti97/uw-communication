#!/usr/bin/env python
import os
import importlib.util, pathlib

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
