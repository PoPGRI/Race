import rospy
# from sensor_msgs.msg import NavSatFix, Imu
from derived_object_msgs.msg import ObjectArray
import pickle 

position_list = []
heading_list = []

def getPosition(data):
    global position_list
    # position_list.append((data.objects[0].pose.position.x, data.objects[0].pose.position.y))
    print(data.objects[0].pose.position)

def on_shutdown():
    pickle.dump(position_list, open("waypoints", "wb"))

if __name__ == "__main__":
    rospy.init_node("waypoint_node")
    subPosition = rospy.Subscriber("/carla/objects", ObjectArray, getPosition)
    # rospy.on_shutdown(on_shutdown)
    rospy.spin()
