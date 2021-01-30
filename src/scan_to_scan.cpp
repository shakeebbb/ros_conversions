#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;
void scan_cb(const sensor_msgs::LaserScan& scanMsg);

ros::Publisher scanPub;
int main(int argc, char** argv){
  ros::init(argc, argv, "scan_to_scan");

  ros::NodeHandle nh("scan_to_scan");

  scanPub = nh.advertise<sensor_msgs::LaserScan>("/scan_reorient", 10);
  ros::Subscriber scanSub = nh.subscribe("/scan", 10, scan_cb);

  ros::spin();
};

void scan_cb(const sensor_msgs::LaserScan& scanMsg)
{
  sensor_msgs::LaserScan scanMsgOut = scanMsg;
  scanMsgOut.header.frame_id = "laser_link";

  scanPub.publish(scanMsgOut);
}
