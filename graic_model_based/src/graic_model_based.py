import carla 
import rospy 
import numpy as np
import sys
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl
from simple_pid import PID

from carla_msgs.msg import CarlaCollisionEvent
from carla_msgs.msg import CarlaLaneInvasionEvent
from graic_msgs.msg import LocationInfo, EvaluationInfo, WaypointInfo
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int16, Float32, String

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
        self.f0 = 100.0

    def throttle_curve(self, thr):
        return 0.7 * 9.81 * self.m * thr + self.f0

    def brake_curve(self, brake):
        return 1.0 * 9.81 * self.m * brake

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
        f0 = self.f0
        x,y,u,v,Psi,r = state[0],state[1],state[2],state[3],state[4],state[5]
        f_tra, delta = input[0],input[1]
        # derivatives
        dx = u*np.cos(Psi) - v*np.sin(Psi)
        dy = u*np.sin(Psi) + v*np.cos(Psi)
        du = (f_tra - f1*u - f2*u**2 - f0)/m
        sign = np.sign(u)
        if sign == 0:
            sign = 1
        safe_u = sign * (np.abs(u)+10.)
        dv = -(C_af + C_ar)*v/m/safe_u + (b*C_ar - a*C_af)*r/m/safe_u - u*r + C_af*delta/m
        dPsi = r
        dr = (b*C_ar - a*C_af)*v/Iz/safe_u -(a**2*C_af + b**2*C_ar)*r/Iz/safe_u + a*C_af*delta/Iz
        dot = np.array([dx,dy,du,dv,dPsi,dr])
        return dot

class ModelBasedVehicle:
    def __init__(self, role_name, host, port):
        self.role_name = role_name
        client = carla.Client(host, port)
        self.world = client.get_world()
        self.map = self.world.get_map()
        self.vehicle_dyn = VehicleDynamics()
        self.state = None
        self.ready = False
        self.input = [0, 0]
        self.vehicle = None
        self.speed_control = PID(Kp=1.5,
            Ki=1.5,
            Kd=0.,
            sample_time=0.05,
            output_limits=(0., 1.))
        self.vehicle_control_cmd = CarlaEgoVehicleControl(throttle=0.)
        self.find_ego_vehicle()
        self.init_state()

        subControl = rospy.Subscriber('/carla/%s/vehicle_control_cmd_manual'%role_name, CarlaEgoVehicleControl, self.controlCallback)
        # subControl = rospy.Subscriber('/carla/%s/vehicle_control'%role_name, CarlaEgoVehicleControl, self.controlCallback)
        subAckermann = rospy.Subscriber('/carla/%s/ackermann_cmd'%role_name, AckermannDrive, self.ackermannCallback)
        self.subLaneInvasion = rospy.Subscriber(
            '/carla/%s/lane_invasion' % role_name, CarlaLaneInvasionEvent,
            self.laneCallback)

    def laneCallback(self, data):
        #if self.reachEnd:
        #    return

        for marking in data.crossed_lane_markings:
            if marking is CarlaLaneInvasionEvent.LANE_MARKING_SOLID:
                # Reset 
                print("Resetting when reaching boundary")
                transform = self.map.get_waypoint(
                    self.vehicle.get_location() -
                    15 * self.vehicle.get_transform().get_forward_vector(),
                    project_to_road=True,
                    lane_type=carla.LaneType.Driving).transform
                transform.location.z = 6
                transform.rotation.roll = 0
                transform.rotation.pitch = 0
                self.vehicle.set_transform(transform)
                self.vehicle.set_target_angular_velocity(carla.Vector3D(x=0, y=0))
                self.state[0] = transform.location.x
                self.state[1] = transform.location.y
                self.state[4] = np.deg2rad(transform.rotation.yaw)

    def init_state(self):
        vehicle_transform = self.vehicle.get_transform()
        x = vehicle_transform.location.x
        y = vehicle_transform.location.y
        Psi = np.deg2rad(vehicle_transform.rotation.yaw)
        self.state = [x, y, 0, 0, Psi, 0]

    def find_ego_vehicle(self):
        self.vehicle = None
        while self.vehicle is None:
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    self.vehicle = actor
                    break
        #self.vehicle.set_simulate_physics(True)

    def controlCallback(self, data):
        self.ready = True
        self.vehicle_control_cmd = data
        self.computeInput()

    def computeInput(self):
        if self.vehicle_control_cmd is None:
            return
        throttle = self.vehicle_control_cmd.throttle
        brake = self.vehicle_control_cmd.brake
        steer = self.vehicle_control_cmd.steer
        reverse = self.vehicle_control_cmd.reverse
        if brake > 0:
            if np.abs(self.state[2]) > 0.01:
                self.input[0] = -np.sign(self.state[2]) * self.vehicle_dyn.brake_curve(brake) # brake
            else:
                self.input[0] = self.vehicle_dyn.f0 # stop
        else:
            self.input[0] = self.vehicle_dyn.throttle_curve(throttle)
            if reverse:
                self.input[0] = -self.input[0]
        self.input[1] = steer # FIXME

    def ackermannCallback(self, data):
        self.ready = True
        self.speed_control.setpoint = data.speed
        force = self.speed_control(self.state[2])
        force_range = [-self.vehicle_dyn.brake_curve(1.), self.vehicle_dyn.throttle_curve(1.)]
        force = force_range[0] + force * (force_range[1] - force_range[0])
        self.input[0] = force
        steering_angle = data.steering_angle
        max_steering_angle = np.pi / 3
        self.input[1] = steering_angle / max_steering_angle

    def tick(self, dt):
        # vehicle_transform = self.vehicle.get_transform()
        # self.state[4] = np.deg2rad(vehicle_transform.rotation.yaw)
        self.state = rk4(self.vehicle_dyn.vehicle_dyn, self.state, self.input, dt)
        self.state[4] = np.mod(self.state[4]+np.pi, 2*np.pi) - np.pi

        _,_,u,v,Psi,r = self.state
        # derivatives
        dx = u*np.cos(Psi) - v*np.sin(Psi)
        dy = u*np.sin(Psi) + v*np.cos(Psi)

        v = carla.Vector3D(x = dx, y = dy)
        #self.vehicle.set_target_velocity(v)
        av = carla.Vector3D(z = np.rad2deg(r))
        #self.vehicle.set_target_angular_velocity(av)

        vehicle_transform = self.vehicle.get_transform()
        
        #print(np.sqrt((vehicle_transform.location.x - self.state[0])**2 + (vehicle_transform.location.y-self.state[1])**2))
        #if np.sqrt((vehicle_transform.location.x - self.state[0])**2 + (vehicle_transform.location.y-self.state[1])**2) > 5:
        #if vehicle_transform.location.z > 5:
        #    print("REACH RESETTING")
        #    self.state[0] = vehicle_transform.location.x
        #    self.state[1] = vehicle_transform.location.y
        #    self.state[4] = np.deg2rad(vehicle_transform.rotation.yaw)
        #else:
        vehicle_transform.location.x = self.state[0]# + v.x * dt
        vehicle_transform.location.y = self.state[1]# + v.y * dt
        vehicle_transform.location.z = 0 # + v.y * dt
        vehicle_transform.rotation.yaw = np.rad2deg(self.state[4])
        self.vehicle.set_transform(vehicle_transform)
        self.speed_control.sample_time = dt

def run(role_name, host, port):
    vehicle = ModelBasedVehicle(role_name, host, port)

    fixed_delta_seconds = 0.05

    while not rospy.is_shutdown():
        if vehicle.ready:
            vehicle.tick(fixed_delta_seconds)
            vehicle.ready = False

if __name__ == "__main__":
    rospy.init_node("graic_model_based")
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 2000)
    role_name = rospy.get_param("~role_name", "ego_vehicle")
    run(role_name, host, port)
