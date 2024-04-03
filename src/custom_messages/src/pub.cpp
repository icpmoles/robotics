#include "ros/ros.h"
#include "std_msgs/String.h"
#include "custom_messages/Num.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
	// same code as pub_sub until here
	ros::Publisher chatter_pub = n.advertise<custom_messages::Num>("chatter", 1000);
	// the new node uses a custom num  for our message type
	ros::Rate loop_rate(10);

	int count = 0;
  
  
  	while (ros::ok()){
	    
	    static int i=0;
		i=(i+1)%1000;
		custom_messages::Num msg;
		msg.num =i;  // Num.msg has a num field
		chatter_pub.publish (msg);
		loop_rate.sleep();
  	}

  	return 0;
}
