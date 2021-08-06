#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

ros::Publisher pathPub;
string targetFrameId;

tf2_ros::Buffer tfBuffer;

void path_cb(const nav_msgs::Path&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_transformer");

  ros::NodeHandle nh("path_transformer");

  while(!nh.getParam("target_frame_id", targetFrameId));

  pathPub = nh.advertise<nav_msgs::Path>("path_out", 10);
  ros::Subscriber pathSub = nh.subscribe("path_in", 10, path_cb);

  tf2_ros::TransformListener* tfListenerPtr_ = new tf2_ros::TransformListener(tfBuffer);

  ros::spin();

  return 0;
};

void path_cb(const nav_msgs::Path& pathMsgIn)
{
  nav_msgs::Path pathMsgOut;
  pathMsgOut.header.frame_id = targetFrameId;
  pathMsgOut.header.stamp = ros::Time::now();
  pathMsgOut.poses.resize(pathMsgIn.poses.size());

  if(pathMsgOut.poses.size() < 1)
  {
    pathPub.publish(pathMsgOut);
    return;
  }

  geometry_msgs::TransformStamped pathInToOut;
  try
  {
    pathInToOut = tfBuffer.lookupTransform(targetFrameId, pathMsgIn.header.frame_id, ros::Time(0));
  }
  catch(tf2::TransformException &ex)
	{
		ROS_WARN("%s",ex.what());
    return;
	}

  for(int i=0; i<pathMsgOut.poses.size(); i++)
    tf2::doTransform (pathMsgIn.poses[i].pose, pathMsgOut.poses[i].pose, pathInToOut);
	
	pathPub.publish(pathMsgOut);
}
