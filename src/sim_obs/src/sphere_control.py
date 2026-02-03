#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry

def talker():
    pub = rospy.Publisher('pos_vel', Odometry, queue_size=10)
    rospy.init_node('shpere_control', anonymous=True)

    path = Odometry()
    path.pose.pose.position.x = 5
    path.pose.pose.position.z = 1.2
    move = 0.05 # m/s
    simTime = 10

    input("start")

    rate = rospy.Rate(simTime) # 10hz
    while not rospy.is_shutdown():
        path.pose.pose.position.x -= move
        path.twist.twist.linear.x = -1*move/(1/simTime)

        pub.publish(path)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass