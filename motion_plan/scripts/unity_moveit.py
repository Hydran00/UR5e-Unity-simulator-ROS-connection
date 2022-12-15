#!/usr/bin/env python3

import rospy
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sys

from motion_plan.msg import JointAngles
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
global pub2

def callback(new_joint):
    #Joint_pos=rospy.wait_for_message(topic="/data",topic_type=JointAngles)
    current_joint=move_group.get_current_joint_values()
    pub2.publish(current_joint)
    
    current_joint[0]=new_joint.joint_angles[0]
    current_joint[1]=new_joint.joint_angles[1]
    current_joint[2]=new_joint.joint_angles[2]
    current_joint[3]=new_joint.joint_angles[3]
    current_joint[4]=new_joint.joint_angles[4]
    current_joint[5]=new_joint.joint_angles[5]


    move_group.go(current_joint,wait=False)
    

def listener():
    subscriber = rospy.Subscriber('/data', JointAngles, callback, queue_size=None)
    rospy.sleep(0.05)
    rospy.spin()

if __name__=='__main__':
    pub = rospy.Publisher("/scaled_pos_joint_traj_controller/command", JointTrajectory,queue_size=1000)
    pub2= rospy.Publisher("/real_robot_joint_state", JointAngles,queue_size=None)
    link_names=["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint","wrist_3_joint"]
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group",anonymous=True)

    robot=moveit_commander.RobotCommander()
    #scene=moveit_commander.planning_scene_interface()
    group_name="manipulator"
    move_group=moveit_commander.MoveGroupCommander(group_name)
    move_group.set_max_velocity_scaling_factor(1)
    move_group.set_max_acceleration_scaling_factor(1)
    listener()

        