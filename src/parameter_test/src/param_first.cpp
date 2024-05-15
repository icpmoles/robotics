#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>


int main(int argc, char **argv){
    
	ros::init(argc, argv, "param_first");
	ros::NodeHandle n; // this is a global node handle
	ros::NodeHandle nh_private("~"); // this is a private node handle

	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("global_parameter", 1000); // publish global topic, no node name
	ros::Publisher local_chatter_pub = nh_private.advertise<std_msgs::String>("local_parameter", 1000); // publish topic with node name
	
	std::string name;
	n.getParam("name", name);  //get global param
	
	std::string local_name;
	nh_private.getParam("name", local_name); //get local param
	
	std::string local_name_from_global;
	std::string param_name = ros::this_node::getName() + "/name"; // we build the "path" of the parameter, it works somehow
	ROS_INFO("local param name: %s", param_name.c_str()); // we use the ROS api to retrieve the parameter by providing the path, this can be done local to local but also from other nodes
	n.getParam(param_name, local_name_from_global);  //get local param using global nodehandle

	ros::Rate loop_rate(10);

  
  	while (ros::ok()){
	    
	    	std_msgs::String msg;
    		msg.data = name;
    		
    		std_msgs::String local_msg;
    		local_msg.data = local_name;

    		ROS_INFO("%s", local_name_from_global.c_str());

    		chatter_pub.publish(msg);
    		local_chatter_pub.publish(local_msg);

    		ros::spinOnce();

    		loop_rate.sleep();
    		
  	}


  	return 0;
}
