import os
import sys
import argparse

import subprocess
# from subprocess import DEVNULL, STDOUT, check_call
import os, signal

import rospy

parser = argparse.ArgumentParser(description="")
parser.add_argument('-N', type=int, default='3')
parser.add_argument('--log', type=str, default='/tmp/')

args = parser.parse_args()

rospy.init_node('RaceMain')
rate = rospy.Rate(10)

vehicles = []
for i in range(args.N):
    # in .launch file: spawn vehicle, control bridge, perception module
    v = {}

    v['launch_log'] = args.log+'/hero%d_launch_log.txt'%i

    role_name = 'hero%d'%i
    v['role_name'] = role_name

    with open('objects.json.template', 'r') as f:
        obj = f.read()
    obj = obj.replace('[[role_name]]', role_name)

    init_pose = [0.8,-240+i*10,0,0,0,90]
    v['init_pose'] = init_pose
    obj = obj.replace('[[spawn_point]]', '"x": %f, "y": %f, "z": %f, "roll": %f, "pitch": %f, "yaw": %f'%tuple(init_pose))

    json_file = '/tmp/objects_%s.json'%role_name
    v['json_file'] = json_file
    with open(v['json_file'], 'w') as f:
        f.write(obj)

    cmd = ('roslaunch spawn_vehicle.launch config_file:=%s role_name:=%s &> %s')%tuple([v['json_file'], v['role_name'], v['launch_log']])
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    v['proc_handler'] = subprocess.Popen(cmd, preexec_fn=os.setsid, shell=True, stdin=None, stdout=None, stderr=None, close_fds=True)
    vehicles.append(v)

def shut_down():
    for v in vehicles:
        os.killpg(os.getpgid(v['proc_handler'].pid), signal.SIGTERM)  # Send the signal to all the process groups
rospy.on_shutdown(shut_down)

while not rospy.is_shutdown():
    rate.sleep()
