#!/usr/bin/python3
#
# Send joint values to UR5 using messages
#

from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import rospy
from motion_plan.msg import JointAngles

SIMULATION = rospy.get_param('/unity_ur5e_motion/simulation')
def main():
    topic_name = ""
    rospy.init_node('send_joints')
    pub2= rospy.Publisher("/real_robot_joint_state", JointAngles,queue_size=10)
    if (SIMULATION):
        topic_name += '/pos_joint_traj_controller/command'
    else:
        topic_name += '/scaled_pos_joint_traj_controller/command'
    pub = rospy.Publisher(topic_name,JointTrajectory,queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        temp = JointAngles()
        curr_state = rospy.wait_for_message('/joint_states',JointState)
        if (SIMULATION): 
            temp.joint_angles= ( curr_state.position[2],curr_state.position[1],
            curr_state.position[0],curr_state.position[5],curr_state.position[6],
            curr_state.position[7] )
        else :
            temp.joint_angles= ( curr_state.position[2],curr_state.position[1],
            curr_state.position[0],curr_state.position[3],curr_state.position[4],
            curr_state.position[5] )
        pub2.publish(temp)
        temp =rospy.wait_for_message('/data',JointAngles)
        traj.header.stamp = rospy.Time.now()
        pts = JointTrajectoryPoint()
        pts.positions = [temp.joint_angles[0],temp.joint_angles[1], temp.joint_angles[2],
        temp.joint_angles[3],temp.joint_angles[4],temp.joint_angles[5]]
        pts.time_from_start = rospy.Duration(1.0)

        # Set the points to the trajectory
        traj.points = []
        traj.points.append(pts)
        # Publish the message
        pub.publish(traj)




if __name__ == '__main__':
    try:
        print('\033[1m\033[92mUR5e ready to execute trajectory! :)\033[0m')
        main()
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
