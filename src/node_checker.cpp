#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

#include <ctime>

template<class T>
class node_checker_class
{
private:
	ros::Subscriber rosSub_;
	float expTopicRate_;
	float expTopicRateErr_;
	int nIter_;
	float* topicRate_;
		
public:
	node_checker_class(ros::NodeHandle*)
	{
		rosSub_ = n.subscribe("topic_in", 10, &rosCb, this);
		
		topicRate_ = new float [nIter_];
	}
	
	~node_checker_class()
	{
		delete topicRate_;
	}
	
	void callbackFunction(const typename T::ConstPtr& msgPtr) 
	{
		static iter = 0;
    static time_t tStamp = time(0);
     
    float tDiff = time(0) - tStamp;
    
    if (iter > 0)
    {
    	topicRate_[iter-1] = 1 / tDiff;
    }
    
    nIter_ ++;
    tStamp = time(0);
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "node_checker_node");
	ros::NodeHandle nh("node_checker");
	
	node_checker_class nodeChecker(&nh);
	
	ros::spin();

	return 0;
}
