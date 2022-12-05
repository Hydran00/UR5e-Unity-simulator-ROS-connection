/**
 * @file demo.cpp
 * @author Davide Nardi
 * @brief Demostration of motion plan with random movement of the robot above the board using gazebo simulation
 * @version 0.1
 * @date 2022-06-22
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ros/ros.h"
#include <cstdlib>
#include <motion_plan/JointAngles.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#define SIMULATION true
#define PI 3.14159
#define GRIPPER_MAX_CLOSURE 0.6
#define GRIPPER_MIN_CLOSURE 0.5
#define Z_BASE_LINK 1.79
#define Z_DESK 0.867
#define GRIPPER_LENGHT 0.2
#define TILE_NUM 30

using namespace std;
using namespace Eigen;
using namespace ros;

using namespace boost;

motion_plan::JointAngles joint_target;

static const std::string PLANNING_GROUP_ARM = "manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "endeffector";
moveit::planning_interface::MoveGroupInterface *arm_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;
moveit::planning_interface::MoveGroupInterface::Plan *arm_motion_plan;
moveit::planning_interface::MoveGroupInterface::Plan *gripper_plan;

moveit::core::RobotModelPtr& robot_model;
moveit::core::RobotStatePtr robot_state;
moveit::robot_model_loader::RobotModelLoader* robot_model_loader;
void setup()
{
    arm_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_ARM);
    gripper_group = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP_GRIPPER);
    arm_group->setPlanningTime(5.0);
    arm_group->setMaxVelocityScalingFactor(0.3);
    arm_group->setMaxAccelerationScalingFactor(0.1);
    arm_motion_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    gripper_plan = new moveit::planning_interface::MoveGroupInterface::Plan();
    robot_model_loader = new moveit::robot_model_loader::RobotModelLoader("robot_description");

    robot_model = robot_model_loader.getModel();
    robot_state = new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);
}
void printRED(string s)
{
    cout << "\033[1;92m" << s << "\033[0m\n";
}

void execute_arm_motion_plan()
{
    cout << endl
         << "Motion plan is now executing!";
    arm_group->move();
}

void open_gripper()
{
    if (!SIMULATION) // with this it should work even with real robot
    {
        return;
    }
    gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("open"));
    bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    printRED("Opening gripper");
    gripper_group->move();
}

void close_gripper()
{
    if (!SIMULATION)
    {
        return;
    }
    gripper_group->setJointValueTarget(gripper_group->getNamedTargetValues("close"));
    bool success = (gripper_group->plan(*gripper_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    printRED("Closing gripper");
    gripper_group->move();
}

/**
 * @brief Move above board with random combination of row and column
 *
 * @param n number of motions
 */
bool move_flag = false;
bool reached_target = true;
void SetJointTarget(const motion_plan::JointAngles::ConstPtr &msg)
{
    if(move_flag == true && reached_target == true)
    for (int i = 0; i < 6; i++)
    {
        joint_target.joint_angles[i] = msg->joint_angles[i];
        cout<<"\nMovimento";
    }
    
    
}
void Move_Robot()
{
    moveit::core::RobotStatePtr current_state = arm_group->getCurrentState();
    std::vector<double> joint_group_positions;   
    joint_group_positions ={(double)joint_target.joint_angles[0], (double)joint_target.joint_angles[1],
     (double)joint_target.joint_angles[2],(double) joint_target.joint_angles[3],
      (double)joint_target.joint_angles[4],(double) joint_target.joint_angles[5]};
    arm_group->setJointValueTarget(joint_group_positions);
    bool success = (arm_group->plan(*arm_motion_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

/////////////////////////

int main(int argc, char **args)
{
    ros::init(argc, args, "unity2moveit");
    ros::NodeHandle n;
    ros::AsyncSpinner spinner(1);
    string topicname = "/data";
    cout << "Subscribing to " << topicname<<endl;
    ros::Subscriber sub = n.subscribe(topicname, 1, SetJointTarget);
    setup();
    while (ros::ok)
    {
        if (move_flag == true)
        {
            Move_Robot();
            move_flag = false;
        }
    }
    return 0;
}
