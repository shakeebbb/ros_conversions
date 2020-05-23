#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Publisher posePub;
ros::Publisher posestampedPub;

string frame_id;

void pose_cb(const geometry_msgs::Pose&);
void posestamped_cb(const geometry_msgs::PoseStamped&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_to_posestamped");

  ros::NodeHandle nh("pose_to_posestamped");

  while(!nh.getParam("frame_id", frame_id));

  posePub = nh.advertise<geometry_msgs::PoseStamped>("pose_out", 10);
  posestampedPub = nh.advertise<geometry_msgs::PoseStamped>("posestamped_out", 10);
  
  ros::Subscriber poseSub = nh.subscribe("pose_in", 10, pose_cb);
	ros::Subscriber posestampedSub = nh.subscribe("posestamped_in", 10, posestamped_cb);

  ros::spin();

  return 0;
};

void posestamped_cb(const geometry_msgs::PoseStamped& posestampedMsg)
{
	geometry_msgs::Pose poseMsg;
	poseMsg.position = posestampedMsg.pose.position;
	poseMsg.orientation = posestampedMsg.pose.orientation;
	
	posePub.publish(poseMsg);
}

void pose_cb(const geometry_msgs::Pose& poseMsg)
{
	geometry_msgs::PoseStamped posestampedMsg;
	
	posestampedMsg.header.frame_id = frame_id;
	posestampedMsg.header.stamp = ros::Time::now();
	
	posestampedMsg.pose.position = poseMsg.position;
	posestampedMsg.pose.orientation = poseMsg.orientation;
	
	posestampedPub.publish(posestampedMsg);
}
