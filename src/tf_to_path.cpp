#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

void add_pose_to_path(const geometry_msgs::PoseStamped& pose, nav_msgs::Path& path);
geometry_msgs::PoseStamped transform_to_pose(const geometry_msgs::TransformStamped& transform, const std::string& pose_frame_id);
double dist(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2);

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_to_path");

  ros::NodeHandle nh("tf_to_path");

  string tf_target_frame = "odom";
  string tf_source_frame = "base_link";
  string path_frame_id = "world";
  double dist_sep = 0.5;
  double loop_rate = 0.2;

  ROS_INFO("Waiting for params ...");

  while(!nh.getParam("tf_target_frame", tf_target_frame));
  while(!nh.getParam("tf_source_frame", tf_source_frame));
  while(!nh.getParam("path_frame_id", path_frame_id));
  while(!nh.getParam("distance_seperation", dist_sep));
  while(!nh.getParam("publish_rate_hz", loop_rate));

  ROS_INFO("Params retrieved");

  ros::Publisher pathPub = nh.advertise<nav_msgs::Path>("path_topic", 10);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  nav_msgs::Path pathOut;


  ros::Rate rate(loop_rate);
  while (ros::ok())
  {
    geometry_msgs::TransformStamped current_transform;
    try
    {
      current_transform = tfBuffer.lookupTransform(tf_target_frame, tf_source_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::PoseStamped current_pose = transform_to_pose(current_transform, path_frame_id);

    if ( (pathOut.poses.size() < 1) || (dist(current_pose, pathOut.poses.back()) > dist_sep) )
      add_pose_to_path( current_pose, pathOut );

    pathOut.header.stamp = ros::Time::now();
    pathPub.publish(pathOut);

    rate.sleep();
  }
  return 0;
}

void add_pose_to_path(const geometry_msgs::PoseStamped& pose, nav_msgs::Path& path)
{
  path.header.stamp = pose.header.stamp;
  path.header.frame_id = pose.header.frame_id;

  path.poses.push_back(pose);
}

geometry_msgs::PoseStamped transform_to_pose(const geometry_msgs::TransformStamped& transform, const std::string& pose_frame_id)
{
  geometry_msgs::PoseStamped poseMsg;

  poseMsg.header.stamp = transform.header.stamp;
  poseMsg.header.frame_id = pose_frame_id;
    
  poseMsg.pose.position.x = transform.transform.translation.x;
  poseMsg.pose.position.y = transform.transform.translation.y;
  poseMsg.pose.position.z = transform.transform.translation.z;

  poseMsg.pose.orientation.x = transform.transform.rotation.x;
  poseMsg.pose.orientation.y = transform.transform.rotation.y;
  poseMsg.pose.orientation.z = transform.transform.rotation.z;
  poseMsg.pose.orientation.w = transform.transform.rotation.w;

  return poseMsg;
}

double dist(const geometry_msgs::PoseStamped& pose1, const geometry_msgs::PoseStamped& pose2)
{
  double x_diff = pose2.pose.position.x - pose1.pose.position.x;
  double y_diff = pose2.pose.position.y - pose1.pose.position.y;
  double z_diff = pose2.pose.position.z - pose1.pose.position.z;

  return sqrt( pow(x_diff,2) + pow(y_diff,2) + pow(z_diff,2) );
}
