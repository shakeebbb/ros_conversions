#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "ncurses.h"

bool operator==(int& op1, std::string&);

int main(int argc, char **argv)
{

  ros::init(argc, argv, "keyboard_to_joy");
  ros::NodeHandle nh("keyboard_to_joy");
  
  ros::Publisher pub = nh.advertise<sensor_msgs::Joy>("joy_out", 100);
  
  sensor_msgs::Joy joyMsg;
  std::vector<std::string> buttonMap;
  std::vector<std::string> axisMap;
  float axisSensitivity;
  
  while(!nh.getParam("buttons", buttonMap))
  ROS_WARN_THROTTLE(1, "Waiting for button param ...");
  while(!nh.getParam("axes", axisMap))
  ROS_WARN_THROTTLE(1, "Waiting for axis param ...");
  while(!nh.getParam("axes_sensitivity", axisSensitivity))
  ROS_WARN_THROTTLE(1, "Waiting for axis param ...");

  joyMsg.buttons.resize(buttonMap.size(), 0);
  joyMsg.axes.resize(axisMap.size()/2, 0.0);
  
  initscr();
  clear();
	noecho();
	raw();
	keypad(stdscr, TRUE);
	nodelay(stdscr, TRUE);
  
  float loopRate = 10;
  ros::Rate rosLoop(loopRate);
  
  while(ros::ok())
  {  	
  	int key = getch();
  	bool isMatch = false;
  	
  	for (int i=0; i<buttonMap.size(); i++)
  	{
  		if (key == buttonMap[i])
			{
				joyMsg.buttons[i] = 1;
				isMatch = true;
			}
  	}
  	
  	for (int i=0; i<axisMap.size(); i++)
  	{
  		if (key == axisMap[i])
  		{
  			if (i%2)
  			joyMsg.axes[i/2] += axisSensitivity / loopRate;
  			else
  			joyMsg.axes[i/2] -= axisSensitivity / loopRate;
  			
  			if (joyMsg.axes[i/2] > 1)
  			joyMsg.axes[i/2] = 1;
  			
  			if (joyMsg.axes[i/2] < -1)
  			joyMsg.axes[i/2] = -1;
  			
  			isMatch = true;
  		}
  	}  	

		if (!isMatch)
		{
			std::fill(joyMsg.buttons.begin(), joyMsg.buttons.end(), 0);
			std::fill(joyMsg.axes.begin(), joyMsg.axes.end(), 0);
		}

		joyMsg.header.stamp = ros::Time::now();
		pub.publish(joyMsg);
  			
  	if (key == int('q'))
  	break;
  	
  	flushinp();
  	rosLoop.sleep();
  	//std::cout << std::getchar() << std::endl;
  }
  
  endwin();
  
  return 0;
}

bool operator==(int& op1, std::string& op2)
{
	if (op1 == ERR)
	return false;
	
	if (op2 == "KEY_DOWN")
	return (op1 == KEY_DOWN);
			
	if (op2 == "KEY_UP")
	return (op1 == KEY_UP);
	
	if (op2 == "KEY_LEFT")
	return (op1 == KEY_LEFT);
		
	if (op2 == "KEY_RIGHT")
	return (op1 == KEY_RIGHT);
			
	if (op2 == "KEY_HOME")
	return (op1 == KEY_HOME);
	
	if (op2 == "KEY_BACKSPACE")
	return (op1 == KEY_BACKSPACE);
			
	if (op2 == "KEY_F(0)")
	return (op1 == KEY_F(0));
			
	if (op2 == "KEY_F(1)")
	return (op1 == KEY_F(1));
			
	if (op2 == "KEY_F(2)")
	return (op1 == KEY_F(2));
			
	if (op2 == "KEY_DC")
	return (op1 == KEY_DC);
	
	if (op2 == "KEY_IC")
	return (op1 == KEY_IC);
			
	if (op2 == "KEY_ENTER")
	return (op1 == KEY_ENTER);
	
	return (op1 == int(op2.c_str()[0]));
}
