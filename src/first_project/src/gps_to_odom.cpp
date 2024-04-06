#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>



void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg, bool *toInit, ros::NodeHandle nh){
    //print out the received lat
    ROS_INFO("Latitude: [%f]", msg->latitude); 
    
    if (*toInit==true) {
        ROS_INFO("Initializing Datum");
        nh.setParam("/lat_zero", msg->latitude);
        nh.setParam("/lon_zero", msg->longitude);
        nh.setParam("/alt_zero", msg->altitude);
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
        ROS_INFO("Initial datum already set"); 
    } else {
        datumToInit = true;
        ROS_INFO("Initial datum is going to be aquired by GPS data"); 
    }
    // subscribe to gps data
    // ros::Subscriber gps_sub = nh.subscribe("fix", 1, gpsCallback);
    // the callback functions also gets the bool if it's supposed to set the datum or not
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> ("fix", 1, boost::bind(gpsCallback, _1, &datumToInit, nh));
    datumToInit=false; //we assume that the callback is going to do its job somehow

    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 5);



	ros::Rate loop_rate(30); 

   

  
  	while (ros::ok()){
	    

        ros::spinOnce();

        loop_rate.sleep();
    		
  	}


  	return 0;
}
