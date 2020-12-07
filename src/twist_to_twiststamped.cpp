#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

using namespace std;

ros::Publisher twistPub;
ros::Publisher twiststampedPub;

string frame_id;

void twist_cb(const geometry_msgs::Twist&);
void twiststamped_cb(const geometry_msgs::TwistStamped&);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "twist_to_twiststamped");

  ros::NodeHandle nh("twist_to_twiststamped");

  while(!nh.getParam("frame_id", frame_id));

  twistPub = nh.advertise<geometry_msgs::Twist>("twist_out", 10);
  twiststampedPub = nh.advertise<geometry_msgs::TwistStamped>("twiststamped_out", 10);
  
  ros::Subscriber twistSub = nh.subscribe("twist_in", 10, twist_cb);
	ros::Subscriber twiststampedSub = nh.subscribe("twiststamped_in", 10, twiststamped_cb);

  ros::spin();

  return 0;
};

void twiststamped_cb(const geometry_msgs::TwistStamped& twiststampedMsg)
{
	geometry_msgs::Twist twistMsg;
	twistMsg.linear = twiststampedMsg.twist.linear;
	twistMsg.angular = twiststampedMsg.twist.angular;
	
	twistPub.publish(twistMsg);
}

void twist_cb(const geometry_msgs::Twist& twistMsg)
{
	geometry_msgs::TwistStamped twiststampedMsg;
	
	twiststampedMsg.header.frame_id = frame_id;
	twiststampedMsg.header.stamp = ros::Time::now();
	
	twiststampedMsg.twist.linear = twistMsg.linear;
	twiststampedMsg.twist.angular = twistMsg.angular;
	
	twiststampedPub.publish(twiststampedMsg);
}
