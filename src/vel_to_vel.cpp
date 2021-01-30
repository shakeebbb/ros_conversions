#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace std;
void twist_cb(const geometry_msgs::Twist&);

geometry_msgs::Twist currTwist;

int main(int argc, char** argv){
  ros::init(argc, argv, "vel_to_vel");

  ros::NodeHandle nh("vel_to_vel");

  ros::Publisher velPub = nh.advertise<geometry_msgs::Twist>("twist_out", 10);
  ros::Subscriber velSub = nh.subscribe("twist_in", 10, twist_cb);

  double rateParam;
  while(!nh.getParam("output_rate_hz", rateParam));

  currTwist.linear.x = 0;
  currTwist.linear.y = 0;
  currTwist.linear.z = 0;
  currTwist.angular.x = 0;
  currTwist.angular.y = 0;
  currTwist.angular.z = 0;

  ros::Rate loopRate(rateParam);
  while(ros::ok())
  {
    velPub.publish(currTwist);

    ros::spinOnce();
    loopRate.sleep();
  }
};

void twist_cb(const geometry_msgs::Twist& twistMsg)
{
  currTwist = twistMsg;
}
