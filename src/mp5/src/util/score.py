import rospy 
import numpy as np
from trackpoint_list import TrackPoints
from gazebo_model_collision_plugin.msg import Contact
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, GetModelStateResponse 
import time


class Scoring:

    def __init__(self):
        self.subCollision = rospy.Subscriber('/gazebo/base_collision', Contact, self.collisionCallback)
        trackPts = TrackPoints()
        self.trackPoints = trackPts.getWayPoints()
        self.reachedPoints = []
        self.speedList = []
        self.hitObjects = set()
        self.deviationCount = 0
        self.score = 0

        self.run()

    def getModelState(self):
        # Get the current state of the vehicle
        # Input: None
        # Output: ModelState, the state of the vehicle, contain the
        #   position, orientation, linear velocity, angular velocity
        #   of the vehicle
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            serviceResponse = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            modelState = serviceResponse(model_name="gem")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: "+str(exc))
            modelState = GetModelStateResponse()
            modelState.success = False
        return modelState

    def collisionCallback(self, data):
        if str(data.objects_hit[0]) in self.hitObjects:
            return
        print("Collision Detected with ", data.objects_hit[0])
        self.hitObjects.add(str(data.objects_hit[0]))


    def getClosedWaypoint(self):
        currState = self.getModelState()
	if not currState.success:
	    return
        x = currState.pose.position.x
        y = currState.pose.position.y 
        closedIdx = 0
        closed = [0, 0, 10000]

        for idx, trackPoint in enumerate(self.trackPoints):
            tx = trackPoint[0]
            ty = trackPoint[1]
            dist = np.sqrt((x-tx)*(x-tx) + (y-ty)*(y-ty))
            if dist < closed[2]:
                closed = [tx, ty, dist]
                closedIdx = idx
        
        distanceToX = abs(x - closed[0])
        distanceToY = abs(y - closed[1])


        vx = currState.twist.linear.x
        vy = currState.twist.linear.y
        v = np.sqrt(vx*vx + vy*vy)
        if v > 0.5:
            self.speedList.append(v)

        if distanceToX < 4 and distanceToY < 4 and [closed[0], closed[1]] not in self.reachedPoints:
            self.reachedPoints.append([closed[0], closed[1]])
            print("Reached ", closed[0], closed[1])
            vBar = np.average(self.speedList)
            self.speedList = []
            self.score += vBar
            print("Current Score: ", self.score)

        return closed[2]

    def isDeviating(self):
        closedDist = self.getClosedWaypoint()
        if closedDist > 6:
            self.deviationCount
            print("car is deviating")

    def onShutdown(self):
        print("Final Score: ")
        if len(self.hitObjects) != 0:
            self.score -= 100
        if self.deviationCount < 400 and self.deviationCount != 0:
            self.score -= 50 * (self.deviationCount/100)
        elif self.deviationCount > 400:
            self.score -= 200
        print(self.score)


    def run(self):
        rate = rospy.Rate(100)  # 100 Hz    
        rospy.on_shutdown(self.onShutdown)
        while not rospy.is_shutdown():
            self.isDeviating() 
                
            rate.sleep()

if __name__ == "__main__":
    print("Start calculating score!")
    rospy.init_node("ScoringNode")
    try:
        Scoring()
    except rospy.exceptions.ROSInterruptException:
        pass


