import carla 
import rospy 
import numpy as np
import sys
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from simple_pid import PID

def rk4(dyn, state, input, dt):
    state = np.array(state)
    input = np.array(input)
    F1 = dt*dyn(state,input)
    F2 = dt*dyn(state + F1/2,input)
    F3 = dt*dyn(state + F2/2,input)
    F4 = dt*dyn(state + F3,input)
    state_next = state + 1.0/6.0*(F1 + 2.0*F2 + 2.0*F3 + F4)
    return state_next.tolist()

class VehicleDynamics(object):
    def __init__(self):
        super(VehicleDynamics, self).__init__()
        self.m = 1500.0

    def throttle_curve(self, thr):
        return 0.7 * 9.81 * self.m * thr

    def vehicle_dyn(self, state, input):
        # INPUTS: state = [x,y,u,v,Psi,r] --- logitude velocity, lateral velocity, yaw angle and yaw angular velocity
        #         input = [f_tra, delta] --- traction force and steering input
        # constants
        a = 1.14 # distance to front axie
        L = 2.54
        m = self.m # mass
        Iz = 2420.0
        C_af = 44000.0*2
        C_ar = 47000.0*2
        b = L-a
        f1 = 0.1
        f2 = 0.01
        f0 = 100.0
        x,y,u,v,Psi,r = state[0],state[1],state[2],state[3],state[4],state[5]
        f_tra, delta = input[0],input[1]
        # derivatives
        dx = u*np.cos(Psi) - v*np.sin(Psi)
        dy = u*np.sin(Psi) + v*np.cos(Psi)
        du = (f_tra - f1*u - f2*u**2 - f0)/m;
        dv = -(C_af + C_ar)*v/m/(u+1e-3) + (b*C_ar - a*C_af)*r/m/(u+1e-3) - u*r + C_af*delta/m
        dPsi = r
        dr = (b*C_ar - a*C_af)*v/Iz/(u+1e-3) -(a**2*C_af + b**2*C_ar)*r/Iz/(u+1e-3) + a*C_af*delta/Iz
        return np.array([dx,dy,du,dv,dPsi,dr])

class ModelBasedVehicle:
    def __init__(self, role_name):
        # subControl = rospy.Subscriber('/carla/%s/vehicle_control_cmd_manual'%role_name, CarlaEgoVehicleControl, self.controlCallback)
        subControl = rospy.Subscriber('/carla/%s/vehicle_control'%role_name, CarlaEgoVehicleControl, self.controlCallback)
        subAckermann = rospy.Subscriber('/carla/%s/ackermann_control'%role_name, AckermannDrive, self.ackermannCallback)

        self.role_name = role_name
        client = carla.Client('localhost', 2000)
        self.world = client.get_world()
        self.vehicle_dyn = VehicleDynamics()
        self.state = None
        self.input = [0, 0]
        self.vehicle = None
        self.speed_control = PID(Kp=1.0,
            Ki=0.1,
            Kd=0.05,
            sample_time=0.05,
            output_limits=(0., 1.))
        self.find_ego_vehicle()
        self.init_state()

    def init_state(self):
        vehicle_transform = self.vehicle.get_transform()
        x = vehicle_transform.location.x
        y = vehicle_transform.location.y
        Psi = np.deg2rad(vehicle_transform.rotation.yaw)
        self.state = [x, y, 0, 0, Psi, 0]

    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break
        self.vehicle.set_simulate_physics(False)

    def controlCallback(self, data):
        thr = data.throttle
        ste = data.steer
        print(thr, ste)
        self.input[0] = self.vehicle_dyn.throttle_curve(thr)
        self.input[1] = ste # FIXME

    def ackermannCallback(self, data):
        self.speed_control.setpoint = data.speed
        self.input[0] = self.vehicle_dyn.throttle_curve(self.speed_control(self.state[2]))
        steering_angle = data.steering_angle
        max_steering_angle = np.pi / 3
        self.input[1] = steering_angle / max_steering_angle

    def tick(self, dt):
        self.state = rk4(self.vehicle_dyn.vehicle_dyn, self.state, self.input, dt)
        vehicle_transform = self.vehicle.get_transform()
        vehicle_transform.location.x = self.state[0]
        vehicle_transform.location.y = self.state[1]
        vehicle_transform.rotation.yaw = np.rad2deg(self.state[4])
        self.vehicle.set_transform(vehicle_transform)
        print(vehicle_transform)
        self.speed_control.sample_time = dt

def main(role_name):
    vehicle = ModelBasedVehicle(role_name)

    freq = 20
    rate = rospy.Rate(freq)

    while not rospy.is_shutdown():
        vehicle.tick(1./freq)
        rate.sleep()

if __name__ == "__main__":
    role_name = sys.argv[1]
    rospy.init_node("model_based_node_%s"%role_name)
    main(role_name)
