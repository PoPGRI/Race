import rospy
# from sensor_msgs.msg import NavSatFix, Imu
# from derived_object_msgs.msg import ObjectArray
from popgri_msgs.msg import LocationInfo
import pickle 

position_list = []
heading_list = []

def getPosition(data):
    global position_list
    position_list.append((data.location.x, data.location.y))
    print(data.location)

def on_shutdown():
    pickle.dump(position_list, open("waypoints", "wb"))

if __name__ == "__main__":
    rospy.init_node("waypoint_node")
    subPosition = rospy.Subscriber("/location", LocationInfo, getPosition)
    rospy.on_shutdown(on_shutdown)
    rospy.spin()
