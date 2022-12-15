Move folders "motion_plan" and "ros_tcp_endpoint" under your /src folder inside the catkin workspace and build the workspace.  
Open project UR5e_UNITY in Unity Hub.  
Then build JointAngles.msg and Gripper_cmd.msg in Unity using Robotics tab (Robotics-> Generate ROS Message).  
Set your IP in Robotics-> ROS Setting  
Launch the entire project with:
```
roslaunch motion_plan ur5e_unity.launch
```

Now you should be able to see /data and /gripper_cmd topic being published (rostopic echo /data) when entering game mode (or after building for oculus inside the application).  Launching gazebo simulation, robot will move according to the topic published:
```
/pos_joint_traj_controller/command
```
or if you are using a real UR5e
```
scaled_pos_joint_traj_controller/command
```
