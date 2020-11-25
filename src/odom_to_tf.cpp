#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

void odom_cb(const nav_msgs::Odometry&);

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_to_tf");

  ros::NodeHandle nh("odom_to_tf");

  ros::Subscriber odomSub = nh.subscribe("odom_topic", 10, odom_cb);
  ros::spin();
  return 0;
};


void odom_cb(const nav_msgs::Odometry& msg)
{
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = msg.header.frame_id;
  transformStamped.child_frame_id = msg.child_frame_id;

  transformStamped.transform.translation.x = msg.pose.pose.position.x;
  transformStamped.transform.translation.y = msg.pose.pose.position.y;
  transformStamped.transform.translation.z = msg.pose.pose.position.z;

  transformStamped.transform.rotation = msg.pose.pose.orientation;

  br.sendTransform(transformStamped);
}
