Move folders "motion_plan" and "ros_tcp_endpoint" under your /src folder inside the catkin workspace and build the workspace.  
Open project UR5e_UNITY in Unity Hub.  
Then build JointAngles.msg in Unity using Robotics tab (Robotics-> Generate ROS Message).  
Set your IP in Robotics-> ROS Setting  
Launch the ROS-TCP-endpoint (see Unity Hub tutorial) with 'roslaunch motion_plan ur5e_unity.launch tcp_ip:=<same ip in unity robotics>'  
Now you should be able to see /data topic being published (rostopic echo /data).  
