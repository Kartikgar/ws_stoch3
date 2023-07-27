#!/usr/bin/env python2.7
import rospy

import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
 
def handle_turtle_pose(msg):

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z
    orientation = msg.pose.pose.orientation


    br = tf.TransformBroadcaster()
    odom_trans = TransformStamped()
    current_time = rospy.Time.now()
    print("transform published ")


    odom_trans.header.stamp = current_time
    odom_trans.header.frame_id = "odom"
    odom_trans.child_frame_id = "base_link"

    odom_trans.transform.translation.x = x
    odom_trans.transform.translation.y = y
    odom_trans.transform.translation.z = z
    odom_trans.transform.rotation = orientation
    br.sendTransform((x,y,z),(orientation.x, orientation.y, orientation.z, orientation.w), rospy.Time.now(), 'base_link', 'odom') 
                    #rospy.Time.now(),
                    #'odom', 'base_link')
                                        
if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    rospy.Subscriber('/odom',Odometry, handle_turtle_pose, queue_size=10)
    rospy.spin()