import carla 
import rospy 
import numpy as np
from popgri_msgs.msg import LocationInfo, LaneList
import math

class ModelBasedVehicle:
    def __init__(self, role_name, world):
        # role_name = "hero0"
        subLocation = rospy.Subscriber("/carla/%s/location"%role_name, LocationInfo, self.locationCallback)
        subLaneWaypoints = rospy.Subscriber("/carla/%s/lane_waypoints"%role_name, LaneList, self.waypointCallback)

        self.role_name = role_name
        self.world = world
        self.vehicle = None
        self.location = None 
        self.rotation = None 
        self.velocity = None
        self.waypoint = None

        self.find_ego_vehicle()

    def find_ego_vehicle(self):
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == self.role_name:
                self.vehicle = actor
                break
        self.vehicle.set_simulate_physics(False) 
        
    def locationCallback(self, data):
        self.location = (data.location.x, data.location.y, data.location.z)
        self.rotation = (np.radians(data.rotation.x), np.radians(data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def waypointCallback(self, data):
        self.waypoint = (data.lane_waypoints[-1].location.x, data.lane_waypoints[-1].location.y)

    def vehicle_dyn(self, state,input):
        # INPUTS: state = [x,y,u,v,Psi,r] --- logitude velocity, lateral velocity, yaw angle and yaw angular velocity
        #         input = [f_tra, delta] --- traction force and steering input
        # constants
        a = 1.14 # distance to front axie
        L = 2.54
        m = 1500.0 # mass
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
        dv = -(C_af + C_ar)*v/m/u + (b*C_ar - a*C_af)*r/m/u - u*r + C_af*delta/m
        dPsi = r
        dr = (b*C_ar - a*C_af)*v/Iz/u -(a**2*C_af + b**2*C_ar)*r/Iz/u + a*C_af*delta/Iz
        return np.array([dx,dy,du,dv,dPsi,dr])

def rk4(dyn, state, input, dt, param):
    state = np.array(state)
    input = np.array(input)
    F1 = dt*dyn(state,input)
    F2 = dt*dyn(state + F1/2,input)
    F3 = dt*dyn(state + F2/2,input)
    F4 = dt*dyn(state + F3,input)
    state_next = state + 1.0/6.0*(F1 + 2.0*F2 + 2.0*F3 + F4)
    return state_next.tolist()

def main(role_name):
    client = carla.Client('localhost', 2000)
    # client.set_timeout(2.0)
    world = client.get_world()
    # map = world.get_map()

    # settings = world.get_settings()

    # use synchronous_mode
    dt = 0.01
    # settings.fixed_delta_seconds = dt
    # settings.synchronous_mode = True
    # world.apply_settings(settings)

    vehicle = ModelBasedVehicle(role_name, world)

    while not vehicle.vehicle or not vehicle.location:
        continue 

    x = vehicle.location[0]
    y = vehicle.location[1] 
    print(vehicle.location)
    u = 20
    v = 0
    Psi = vehicle.rotation[2]
    r = 0
    ud = 20
    param = {}

    rate = rospy.Rate(100)

    # get nearest waypoint
    # wp_current = vehicle.location
    # wp_next = wp_current.next_until_lane_end(5)[0]

    wp_next = vehicle.waypoint
    while not wp_next:
        wp_next = vehicle.waypoint

    while not rospy.is_shutdown():
        # get nearest waypoint
        wp_current = vehicle.location
        # wp_next = wp_current.next_until_lane_end(5)[0]
        # wp_next = vehicle.waypoint

        if not wp_current or not wp_next:
            continue
        
        # print("Current: ", wp_current)
        # print("Next: ", wp_next)
        dy = wp_next[1] - vehicle.location[1];
        dx = wp_next[0] - vehicle.location[0];

        if abs(dy) < 5 and abs(dx) < 5:
            wp_next = vehicle.waypoint
        # propogate the vehicle position
        vehicle_transform = vehicle.vehicle.get_transform()
        # vehicle_transform.location +=  vehicle_transform.get_forward_vector()*u*dt + vehicle_transform.get_right_vector()*v*dt
        vehicle_transform.location.x = x
        vehicle_transform.location.y = y
        vehicle_transform.location.z = vehicle.location[2]
        vehicle_transform.rotation.yaw = np.rad2deg(Psi);
        vehicle.vehicle.set_transform(vehicle_transform)

        # print("the yaw is %d",np.rad2deg(Psi))
        # control inputs
        acc = -500*(u-ud);
        # steer = -0.1*(vehicle_transform.rotation.yaw-wp_current.transform.rotation.yaw)/180.0*math.pi; # steer > 0 the vehicle is turning right
        steer = -0.5*(np.deg2rad(vehicle_transform.rotation.yaw)-math.atan2(dy,dx))
        steer =  min(math.pi/2.0, max(-math.pi/2.0, steer))
        # steer = 0.2
        # propogate the vehicle velocity
        # print("Before: ", [x,y,u,v,Psi,r])
        [x,y,u,v,Psi,r] = rk4(vehicle.vehicle_dyn, [x,y,u,v,Psi,r], [acc,steer], dt, param)
        # print("After: ", [x,y,u,v,Psi,r])
        # print("the yaw difference is %d",np.deg2rad(vehicle_transform.rotation.yaw)-math.atan2(dy,dx))
        # print("the steering is %d",steer)

        rate.sleep()

if __name__ == "__main__":
    rospy.init_node("model_based_node")
    role_name = "hero0" # NOTE hardcode for now

    main(role_name)
    