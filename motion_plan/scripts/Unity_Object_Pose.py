#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose

if __name__ == "__main__":
    rospy.init_node("position_sender",anonymous=True)
    pub = rospy.Publisher("/cup_position", Pose,queue_size=10)
    #compute vision
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose = Pose()
        pose.position.x = -0.15
        pose.position.y = -0.28
        pose.position.z = 1.15
        pose.orientation.w= 1
        pub.publish(pose)
        print('.')
        rate.sleep()