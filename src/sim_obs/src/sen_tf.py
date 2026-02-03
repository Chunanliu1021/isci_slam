#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import geometry_msgs.msg

import tf
import tf2_ros

def handle_turtle_pose(msg):
    # br = tf2_ros.TransformBroadcaster()
    # t = geometry_msgs.msg.TransformStamped()
    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = "world"
    # t.transform.translation.x = 0
    # t.transform.translation.y = 0
    # t.transform.translation.z = 0

    # t.transform.rotation.x = 0
    # t.transform.rotation.y = 0
    # t.transform.rotation.z = 0
    # t.transform.rotation.w = 1
    # t.child_frame_id = "odom"

    # br.sendTransform(t)

    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation.x = msg.pose.pose.orientation.x
    t.transform.rotation.y = msg.pose.pose.orientation.y
    t.transform.rotation.z = msg.pose.pose.orientation.z
    t.transform.rotation.w = msg.pose.pose.orientation.w
    t.child_frame_id = "base_link"

    # br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')

    rospy.Subscriber('/ground_truth',
                     Odometry,
                     handle_turtle_pose,
                     )
    rospy.spin()
