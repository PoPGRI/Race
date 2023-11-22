import os 
import subprocess
import time
import pickle 

map = "t3" # t1_triple, t2_triple, t3, t4, shanghai_intl_circuit

controller_process = subprocess.Popen("python3 automatic_control_GRAIC.py --sync -m {}".format(map), shell=True)
