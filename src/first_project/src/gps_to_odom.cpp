#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>


// class gpsCallback {
//     public:

//     void chatterCallback(const std_msgs::String::ConstPtr& msg){
        
//     ROS_INFO("I heard: [%s]", msg->data.c_str()); //print out we receive a message
//     }
// private:

// }

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, bool toInit){
    //print out the received lat
    ROS_INFO("Latitude: [%s]", std::to_string(msg->latitude)); 
    if (toInit){
        ROS_INFO("datum to initialize");
    }
}

int main(int argc, char **argv){
    
	ros::init(argc, argv, "gps2odom");

    ros::NodeHandle nh;
    bool datumToInit = false;
    float datum_latitide, datum_longitude, datum_altitude;
    //if the datum has been initialized already populate the fields
    if (nh.hasParam("/lat_zero") and nh.hasParam("/lon_zero") and nh.hasParam("/alt_zero") ) 
    {
        nh.getParam("/lat_zero", datum_latitide);  
        nh.getParam("/lon_zero", datum_longitude);
        nh.getParam("/alt_zero", datum_altitude);
        ROS_INFO("Initial datum acquired by parameters"); 
    } else {
        datumToInit = true;
        ROS_INFO("Initial datum is going to be aquired by GPS data"); 
    }
    // subscribe to gps data
    // ros::Subscriber gps_sub = nh.subscribe("fix", 1, gpsCallback);
    ros::Subscriber gps_sub = nh.subscribe("fix", 1, boost::bind(gpsCallback, _1, datumToInit));


    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 5);



	// ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("global_parameter", 1000); // publish global topic, no node name

	// std::string name;
	// n.getParam("name", name);  //get global param
	
	// std::string local_name;
	// nh_private.getParam("name", local_name); //get local param
	
	// std::string local_name_from_global;
	// std::string param_name = ros::this_node::getName() + "/name"; // we build the "path" of the parameter, it works somehow
	// ROS_INFO("local param name: %s", param_name.c_str()); // we use the ROS api to retrieve the parameter by providing the path, this can be done local to local but also from other nodes
	// n.getParam(param_name, local_name_from_global);  //get local param using global nodehandle

	// ros::Rate loop_rate(10); nh.hasParam("/lat_zero") and nh.hasParam("/lon_zero") and nh.hasParam("/alt_zero") ) 

  
  	while (ros::ok()){
	    
        if (datumToInit==true) {
            nh.setParam("/lat_zero", gps_sub);
            nh.setParam("/lon_zero", 5);
            nh.setParam("/alt_zero", 5);
            datumToInit=false;
        } else {
            datumToInit=false;
        }

        // std_msgs::String msg;
        // msg.data = name;
        
        // std_msgs::String local_msg;
        // local_msg.data = local_name;

        // ROS_INFO("%s", local_name_from_global.c_str());

        // chatter_pub.publish(msg);
        // local_chatter_pub.publish(local_msg);

        ros::spinOnce();

        loop_rate.sleep();
    		
  	}


  	return 0;
}
