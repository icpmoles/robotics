#include "ros/ros.h"
#include "std_msgs/String.h"



int main(int argc, char **argv){

	ros::init(argc, argv, "talker"); // intialize the node with a meaningful name
	
	//, ros::init_options::AnonymousName); // intialize the node with an "automatic random generated" name without needing to remap at every start in case of conflict
	ros::NodeHandle n; //create nodeHandle

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1);
	//create publisher object with node.advertise with a type string and name "chatter" and 1 as size of the buffer of the publisher (1 is good most of the time)(can be increased in case your calculations take too much time)

	ros::Rate loop_rate(10);
	// running frequency of the loop at 10Hz

	int count = 0;

  	while (ros::ok()){ //standard ros loop, check if ROS is working, exit otherwise

	    	std_msgs::String msg; 

                msg.data = "hello world!"; //ros msg object only has data field

    		ROS_INFO("%s", msg.data.c_str()); //standard ROS logging/debugging message
			// we just log the content of our msg

    		chatter_pub.publish(msg);
			// we just publish the content of our msg

    		ros::spinOnce();
			// handles all the other generic ros tasks like callbacks timers etc, useful for more complex
			// code

    		loop_rate.sleep(); //just waits until the next 10Hz loop, better this way 
			// instead of the built in sleep

		
    		++count;
  	}


  	return 0;
}
