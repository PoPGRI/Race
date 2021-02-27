import os
import sys
# import argparse

import subprocess
# from subprocess import DEVNULL, STDOUT, check_call
import os, signal

import rospy

def Spawn(N, log, set_spectator=True):

    rate = rospy.Rate(10)

    os.chdir(os.path.dirname(__file__))
    cwd = os.getcwd()

    vehicles = []
    for i in range(N):
        # in .launch file: spawn vehicle, control bridge, perception module
        v = {}

        v['launch_log'] = log+'/hero%d_launch_log.txt'%i

        role_name = 'hero%d'%i
        v['role_name'] = role_name

        with open('objects.json.template', 'r') as f:
            obj = f.read()
        obj = obj.replace('[[role_name]]', role_name)

        init_pose = [164,11+i*10,4,0,0,-180]
        v['init_pose'] = init_pose
        obj = obj.replace('[[spawn_point]]', '"x": %f, "y": %f, "z": %f, "roll": %f, "pitch": %f, "yaw": %f'%tuple(init_pose))

        json_file = '/tmp/objects_%s.json'%role_name
        v['json_file'] = json_file
        with open(v['json_file'], 'w') as f:
            f.write(obj)

        cmd = ('roslaunch race spawn_vehicle.launch config_file:=%s role_name:=%s &> %s')%tuple([v['json_file'], v['role_name'], v['launch_log']])
        # The os.setsid() is passed in the argument preexec_fn so
        # it's run after the fork() and before  exec() to run the shell.
        v['proc_handler'] = subprocess.Popen(cmd, preexec_fn=os.setsid, shell=True, stdin=None, stdout=None, stderr=None, close_fds=True)
        vehicles.append(v)

    if set_spectator:
        import carla
        import numpy as np
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        spectator = world.get_spectator()
        center = np.mean([v['init_pose'][:3] for v in vehicles], axis=0)
        transform = carla.Transform(carla.Location(x=center[0], y=-center[1], z=center[2] + 40),
                                    carla.Rotation(pitch=-89, yaw=-62.5))
        spectator.set_transform(transform)

    def shut_down():
        for v in vehicles:
            os.killpg(os.getpgid(v['proc_handler'].pid), signal.SIGTERM)  # Send the signal to all the process groups
    rospy.on_shutdown(shut_down)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('RaceMain')
    N = rospy.get_param("~N", 3)
    log = rospy.get_param("~log", '/tmp/')
    import time
    time.sleep(10)
    try: 
        Spawn(N, log)
    except rospy.exceptions.ROSInterruptException:
        rospy.loginfo("RaceMain shut down")