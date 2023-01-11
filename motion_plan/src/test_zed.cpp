#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_msgs/SolidPrimitive.h>
void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Create a publisher for the collision object
  ros::NodeHandle nh;
  ros::Publisher collision_object_pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);

  // Create a collision object message
  moveit_msgs::CollisionObject collision_object;
  collision_object.header = msg->header;
  collision_object.id = "point_cloud_collision_object";
  collision_object.type = shape_msgs::SolidPrimitive
  collision_object.operation = moveit_msgs::CollisionObject::ADD;
  collision_object.primitives.push_back(*msg);

  // Publish the collision object
  collision_object_pub.publish(collision_object);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "collision_object_node");

  // Create a subscriber for the point cloud
  ros::NodeHandle nh;
  ros::Subscriber point_cloud_sub = nh.subscribe("point_cloud", 1, pointCloudCallback);

  // Spin to process incoming messages
  ros::spin();

  return 0;
}

