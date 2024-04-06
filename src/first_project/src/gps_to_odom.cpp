#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#define A_EQRAD 6378137
#define B_POLRAD 6356752.314
#define E_SQUARED 0.00669437999014
// taken from https://en.wikipedia.org/wiki/Latitude#The_geometry_of_the_ellipsoid

struct ECEF {
  float X, Y, Z;
};

struct NED{
    float N, E, D;
};

float prime_vertical(float lat ){
    float a = A_EQRAD ; // equatorial radius in meters
    //float b = B_POLRAD; // polar radius in meters
    float e2 =  E_SQUARED;
    return a / sqrt(1-e2*pow(sin(lat),2));
}

// https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470099728.app3 C.80 -> C.82
NED ECEFtoNED(ECEF datum, float lat, float lon, ECEF position){ 
    float deltaX=position.X-datum.X;
    float deltaY=position.Y-datum.Y;
    float deltaZ=position.Z-datum.Z;

    NED temp;
    temp.N = -cos(lat)*sin(lon)*deltaX + 
            (-sin(lat)*sin(lon))*deltaY +
            cos(lon)*deltaZ;
    temp.E = -sin(lat)*deltaX + 
            cos(lon)*deltaY;
    temp.D = -cos(lat)*cos(lon)*deltaX + 
            (-sin(lat)*cos(lon))*deltaY +
            (-sin(lon))*deltaZ; 
    return temp;           
}

ECEF gpsToECEF(float lat, float lon, float alt){
    float pv = prime_vertical(lat);
    float common = (pv + alt)*cos(lat);
    float X = common*cos(lon);
    float Y = common*sin(lon);
    float Z = ((1-E_SQUARED)*pv + alt)*sin(lat);

    // return syntax or something
    ECEF conv;
    conv.X=X;
    conv.Y=Y;
    conv.Z=Z;
    return conv;
}

void gpsCallback(   const sensor_msgs::NavSatFix::ConstPtr& msg, 
                    bool *toInit, 
                    ros::NodeHandle nh, 
                    ros::Publisher ph,
                    ECEF *initFix)
{
    //print out the received lat
    // ROS_INFO("Latitude: [%f]", msg->latitude); 
    float lat = msg->latitude;
    float lon = msg->longitude;
    float alt = msg->altitude;

    // gps to ECEF
    ECEF newPos = gpsToECEF(lat, lon, alt);
    if (*toInit==true) {
        ROS_INFO("Initializing Datum");
        nh.setParam("/lat_zero", lat);
        nh.setParam("/lon_zero", lon);
        nh.setParam("/alt_zero", alt);
        *initFix = gpsToECEF(lat, lon, alt);
        *toInit = false;
    } else {
        // ROS_INFO("Init X: [%f]", initFix->X); 
        // ROS_INFO("DeltaX: [%f]", newPos.X-(initFix->X)); 
        // ROS_INFO("DeltaY: [%f]", newPos.Y-(initFix->Y)); 
        // ROS_INFO("DeltaZ: [%f]", newPos.Z-(initFix->Z)); 
    }

    nav_msgs::Odometry data;
    data.header=msg->header;
    data.header.frame_id= "gps2odom_stamp";
    data.child_frame_id = "gps_transceiver";
    
    // ECEF to NED
    NED deltaNED = ECEFtoNED(*initFix,lat,lon,newPos);

    data.pose.pose.position.x=deltaNED.N;
    data.pose.pose.position.y=deltaNED.E;
    data.pose.pose.position.z=deltaNED.D;
    ROS_INFO("DeltaN: [%f]", deltaNED.N); 
    ROS_INFO("DeltaE: [%f]", deltaNED.E); 
    ROS_INFO("DeltaD: [%f]", deltaNED.D);


    ph.publish(data);

}

int main(int argc, char **argv){
    ECEF initialECEF;
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
        initialECEF=gpsToECEF(datum_latitide,datum_longitude,datum_altitude);
    } else {
        datumToInit = true;
        ROS_INFO("Initial datum is going to be aquired by GPS data"); 
    }

    
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 5);
    // subscribe to gps data
    // ros::Subscriber gps_sub = nh.subscribe("fix", 1, gpsCallback);
    // the callback functions also gets the bool if it's supposed to set the datum or not
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> ("fix", 1, boost::bind(gpsCallback, _1, &datumToInit, nh, odom_pub,&initialECEF));
    // datumToInit=false; //we assume that the callback is going to do its job somehow

	ros::Rate loop_rate(30); 

   

  
  	while (ros::ok()){
	    

        ros::spinOnce();

        loop_rate.sleep();
    		
  	}


  	return 0;
}
