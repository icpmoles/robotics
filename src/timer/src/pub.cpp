#include "ros/ros.h"
#include "std_msgs/String.h"
#include <time.h>


void timerCallback(const ros::TimerEvent& ev){

	ROS_INFO_STREAM("Callback called at time: " <<  ros::Time::now());
	// standard command to get the time inside ROS
}

int main(int argc, char **argv){
    
	ros::init(argc, argv, "timed_talker");
	ros::NodeHandle n;

	ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);
	
	ros::spin();

  	return 0;
}
