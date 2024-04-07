#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#define PI 3.14159265358979323
// taken from https://en.wikipedia.org/wiki/Latitude#The_geometry_of_the_ellipsoid
#define A_EQRAD 6378137
#define B_POLRAD 6356752.31425
#define E_SQUARED 0.00669437999014

struct ECEF {
    double X, Y, Z;
};

struct NED{
    double N = 0, E = 0, D = 0;
    ros::Time timestamp ;
};

double prime_vertical(double lat ){
    double a = A_EQRAD ; // equatorial radius in meters
    //float b = B_POLRAD; // polar radius in meters
    double e2 =  E_SQUARED;
    return a / sqrt(1-e2*pow(sin(lat),2));
}

// https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470099728.app3 C.80 -> C.82
// ECEF coordinates plus latitude and longitude (in radians) of the "landmark reference" as float, and the new position in ECEF
NED ECEFtoNED(ECEF datum, double lat, double lon, ECEF position){ 
    double deltaX=position.X-datum.X;
    double deltaY=position.Y-datum.Y;
    double deltaZ=position.Z-datum.Z;

    NED temp;
    temp.N =-cos(lon)* sin(lat)  *deltaX  
            -sin(lon)* sin(lat)  *deltaY 
                    +  cos(lat) *deltaZ;
    temp.E =   -sin(lon)*deltaX + 
                cos(lon)*deltaY;
    temp.D = -cos(lon)*cos(lat)*deltaX + 
            (-sin(lon)*cos(lat))*deltaY +
                    (-sin(lat))*deltaZ; 
    return temp;           
}

ECEF gpsToECEF(double lat, double lon, double alt){
    double pv = prime_vertical(lat);
    double common = (pv + alt)*cos(lat);
    double X = common*cos(lon);
    double Y = common*sin(lon);
    double Z = ((1-E_SQUARED)*pv + alt)*sin(lat);

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
                    ECEF *initFix,
                    NED *prevPo)
{
    bool logging = false;
    //print out the received lat
    // ROS_INFO("Latitude: [%f]", msg->latitude); 
    // in deg as provided by the GPS fix then they are converted in radians
    double lat = msg->latitude *PI/180;
    double lon = msg->longitude*PI/180;
    double alt = msg->altitude *PI/180;
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
        if (logging){
        ROS_INFO("\nDelta in ECEF");
        ROS_INFO("Init X: [%f]", initFix->X); 
        ROS_INFO("DeltaX: [%f]", newPos.X-(initFix->X)); 
        ROS_INFO("DeltaY: [%f]", newPos.Y-(initFix->Y)); 
        ROS_INFO("DeltaZ: [%f]", newPos.Z-(initFix->Z)); }
    }

    nav_msgs::Odometry data;
    data.header=msg->header;
    data.header.frame_id= "gps2odom_stamp";
    data.child_frame_id = "gps_transceiver";
    // ECEF to NED
    NED actualNED = ECEFtoNED(*initFix,lat,lon,newPos); // NED at t
    actualNED.timestamp=msg->header.stamp; // the NED positione was "acquired" at the same time of the Fix

    data.pose.pose.position.x=actualNED.N;
    data.pose.pose.position.y=actualNED.E;
    data.pose.pose.position.z=actualNED.D;
    
    //calculate heading
    double delta_space=0; //distance between two fixes
    double yaw = 0; // yaw of vehicle fixed to E axis, positive ccw
    double delta_N=0; // difference in 2d NED place
    double delta_E=0;
    ros::Duration delta_T(0.5);
    double diff_t;
    double vel;
    if (msg->header.seq>1)
    {
        if ( (actualNED.timestamp - prevPo->timestamp).toSec() < 10 ){ //in case the delta_T is too crazy (eg reset of bag or startup) let's give an arbitrary value
            delta_T = actualNED.timestamp - prevPo->timestamp;
        }
        diff_t = delta_T.toSec();
        delta_N = actualNED.N - prevPo->N;
        delta_E = actualNED.E - prevPo->E;
        delta_space = sqrt( pow(delta_N,2) + pow(delta_E,2) );
        vel = delta_space/diff_t;
    } // otherwise we simply update 

    *prevPo = actualNED;
    

    data.twist.twist.linear.x=vel;
    data.twist.twist.linear.y=diff_t;
    // ROS_INFO("DeltaT is %f", diff_t);

    if (logging){
    // for debugging we put some ECEF in the odometry topic
    data.pose.pose.orientation.x=newPos.X;
    data.pose.pose.orientation.y=newPos.Y;
    data.pose.pose.orientation.z=newPos.Z;
    // for debugging we put some delta(ECEF) in the odometry twist topic
    data.twist.twist.linear.x=newPos.X-(initFix->X);
    data.twist.twist.linear.y=newPos.Y-(initFix->Y);
    data.twist.twist.linear.z=newPos.Z-(initFix->Z);
    // for debugging we put the initial ECF in the odometry twist topic
    data.twist.twist.angular.x=initFix->X;
    data.twist.twist.angular.y=initFix->Y;
    data.twist.twist.angular.z=initFix->Z;

    ROS_INFO("NED coordinates"); 
    ROS_INFO("DeltaN: [%f]", actualNED.N); 
    ROS_INFO("DeltaE: [%f]", actualNED.E); 
    ROS_INFO("DeltaD: [%f]", actualNED.D);
    }
    

    ph.publish(data);

}

int main(int argc, char **argv){
    ECEF initialECEF;
	ros::init(argc, argv, "gps2odom");

    ros::NodeHandle nh;
    bool datumToInit = false;
    double datum_latitide, datum_longitude, datum_altitude; // LLA in degrees as provided by the GPS fix
    //if the datum has been initialized already populate the fields
    if (nh.hasParam("/lat_zero") and nh.hasParam("/lon_zero") and nh.hasParam("/alt_zero") ) 
    {   
        // if the starting parameter is available we get it;
        nh.getParam("/lat_zero", datum_latitide);  
        nh.getParam("/lon_zero", datum_longitude);
        nh.getParam("/alt_zero", datum_altitude);
        ROS_INFO("Initial datum already set");
        datum_latitide=datum_latitide*PI/180;
        datum_longitude=datum_longitude*PI/180;
        datum_altitude=datum_altitude*PI/180;
        initialECEF = gpsToECEF (datum_latitide,datum_longitude,datum_altitude);
    } else {
        // otherwise we listen to the GPS data and acquire it
        datumToInit = true;
        ROS_INFO("Initial datum is going to be aquired by GPS data"); 
    }

    // starting NED position, it's going to be updated through the execution
    NED prevPoistion;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 5);
    // subscribe to gps data
    // ros::Subscriber gps_sub = nh.subscribe("fix", 1, gpsCallback);
    // the callback functions also gets the bool if it's supposed to set the datum or not
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> ("fix", 3, boost::bind(gpsCallback, _1, &datumToInit, nh, odom_pub,&initialECEF, &prevPoistion));
    // datumToInit=false; //we assume that the callback is going to do its job somehow

	ros::Rate loop_rate(60); 

   

  
  	while (ros::ok()){
	    

        ros::spinOnce();

        loop_rate.sleep();
    		
  	}


  	return 0;
}
