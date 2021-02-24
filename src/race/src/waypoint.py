import rospy
# from sensor_msgs.msg import NavSatFix, Imu
# from derived_object_msgs.msg import ObjectArray
# from popgri_msgs.msg import LocationInfo
import pickle 
import carla
import numpy as np

# position_list = []
# heading_list = []

# def getPosition(data):
#     global position_list
#     position_list.append((data.location.x, data.location.y))
#     print(data.location)

# def on_shutdown():
#     pickle.dump(position_list, open("waypoints", "wb"))

# if __name__ == "__main__":
#     rospy.init_node("waypoint_node")
#     subPosition = rospy.Subscriber("/location", LocationInfo, getPosition)
#     rospy.on_shutdown(on_shutdown)
#     rospy.spin()


def run():
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    actors = world.get_actors()
    map_ = world.get_map()

    # vehicle = None 
    for actor in actors:
        print(actor)
        if 'vehicle' in actor.type_id:
            vehicle = actor 

    # prev_wp = None 
    # wp = None
    # wp0 = None
    # while True:
    #     wp = map_.get_waypoint(vehicle.get_location(),project_to_road=True, lane_type=(carla.LaneType.Driving))
    #     if prev_wp == None or np.sqrt((prev_wp.transform.location.x - wp.transform.location.x)**2 + (prev_wp.transform.location.y - wp.transform.location.y)**2) > 20:
    #         if prev_wp == None:
    #             wp0 = wp
    #         print(wp.transform.location.x, wp.transform.location.y, wp.transform.location.z)
    #         prev_wp = wp 
    #         if np.sqrt((wp0.transform.location.x - wp.transform.location.x)**2 + (wp0.transform.location.y - wp.transform.location.y)**2) < 20 and wp0 != wp:
    #             exit(0)

if __name__ == '__main__':
    run()