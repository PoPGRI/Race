import os 
import subprocess
import time
import pickle 

# Scenario Process will hold until an ego vehicle is available
scenario_process = subprocess.Popen("python3 scenario.py", shell=True, stdout=None)
time.sleep(5)
controller_process = subprocess.Popen("python3 automatic_control_GRAIC_DEMO.py --sync", shell=True)
# controller_process = subprocess.Popen("python3 automatic_control_GRAIC.py", shell=True)
