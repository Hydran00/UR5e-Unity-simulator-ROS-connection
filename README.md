Move folders "motion_plan" and "ros_tcp_endpoint" under your /src folder inside the catkin workspace and build the workspace.  
Open project UR5e_UNITY in Unity Hub.  
Then build JointAngles.msg and Gripper_cmd.msg in Unity using Robotics tab (Robotics-> Generate ROS Message).  
Set your IP in Robotics-> ROS Setting  
Launch the entire project with:
```
roslaunch motion_plan ur5e_unity.launch tcp_ip:=<your_ip>
```
If you are using gazebo you have to set the parameter "simulation" to true
```
roslaunch motion_plan ur5e_unity.launch tcp_ip:=<your_ip> simulation:=true
```

Now you should be able to see /data and /gripper_cmd topic being published when entering game mode (or after building for oculus inside the application).  Launching gazebo simulation, robot will move according to the topic published:
```
/pos_joint_traj_controller/command
```
or if you are using a real UR5e
```
scaled_pos_joint_traj_controller/command
```
Pressing key space in Unity Editor (or key A and B in the app) will open and close gripper. This will trigger the Unity_Gripper.py that will send a script to the robot for the soft robotics gripper.


The following video is an example of manipulation of an object, which is remapped into another object in the virtual world. This is done using ArUco and ROS. More informations in the submodule
https://user-images.githubusercontent.com/93198865/218813911-1af7c823-ce4c-47d8-b82c-875cb647fa24.mp4




Uploading VR_project.mp4…

