#!/usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time
from stoch3_msgs.msg import QuadrupedRobotState


class Stoch3TFBroadcaster:

    def __init__(self):
        self.b = TransformBroadcaster()

        self.robot_state_sub = rospy.Subscriber("robot_state", QuadrupedRobotState, self._robotStateCB, queue_size=1)

    def _robotStateCB(self, data):
        
        position = list([0., 0., 0.])
        position[0] = data.pose.position.x
        position[1] = data.pose.position.y
        position[2] = data.pose.position.z

        orientation = list([0., 0., 0., 1.])
        orientation[0] = data.pose.orientation.x
        orientation[1] = data.pose.orientation.y
        orientation[2] = data.pose.orientation.z
        orientation[3] = data.pose.orientation.w
        
        self.b.sendTransform(position, orientation, Time.now(), "base_link", "world")



if __name__ == "__main__":

    rospy.init_node("stoch3_transform_broadcaster")
    stoch3_tf_broadcaster = Stoch3TFBroadcaster()
    rospy.spin()

