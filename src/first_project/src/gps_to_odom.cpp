#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
// taken from https://en.wikipedia.org/wiki/Latitude#The_geometry_of_the_ellipsoid

#define A_EQRAD 6378137
#define B_POLRAD 6356752.31425
#define E_SQUARED 0.00669437999014

// from https://docs.ros.org/en/melodic/api/inertial_sense/html/ISEarth_8c_source.html
#define POWA2   40680631590769.000      // = pow(6378137.0,2)
#define POWB2   40408299984661.453      // = pow(6356752.31424518,2)
#define POWA2_F 40680631590769.000f     // = pow(6378137.0,2)
#define POWB2_F 40408299984661.453f     // = pow(6356752.31424518,2)
#define ONE_MINUS_F 0.996647189335253 // (1 - f), where f = 1.0 / 298.257223563 is Earth flattening
#define E_SQ  0.006694379990141 // e2 = 1 - (1-f)*(1-f) - square of first eccentricity
#define REQ 6378137.0         // Re - Equatorial radius, m
#define REP 6356752.314245179 // Rp - Polar radius, m
#define E2xREQ 42697.67270717795 // e2 * Re
#define E2xREQdivIFE 42841.31151331153 // e2 * Re / (1 -f)
#define GEQ 9.7803253359        // Equatorial gravity
#define K_GRAV 0.00193185265241 // defined gravity constants
#define K3_GRAV 3.0877e-6       // 
#define K4_GRAV 4.0e-9          //
#define K5_GRAV 7.2e-14         //
 


struct ECEF {
    double X, Y, Z;
};

struct NED{
    double N, E, D;
};

float prime_vertical(float lat ){
    float a = A_EQRAD ; // equatorial radius in meters
    //float b = B_POLRAD; // polar radius in meters
    float e2 =  E_SQUARED;
    return a / sqrt(1-e2*pow(sin(lat),2));
}

// https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470099728.app3 C.80 -> C.82
// ECEF coordinates plus latitude and longitude (in float) of the "landmark reference" as float, and the new position in ECEF
NED ECEFtoNED(ECEF datum, float lat, float lon, ECEF position){ 
    float deltaX=position.X-datum.X;
    float deltaY=position.Y-datum.Y;
    float deltaZ=position.Z-datum.Z;

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

 ECEF llaToECEF(float lat, float lon, float alt ) // (const double *LLA, double *Pe)
 { // from https://docs.ros.org/en/melodic/api/inertial_sense/html/ISEarth_8c_source.html
     //double e = 0.08181919084262;  // Earth first eccentricity: e = sqrt((R^2-b^2)/R^2);
     double Rn, Smu, Cmu, Sl, Cl;
 
     /* Earth equatorial and polar radii 
       (from flattening, f = 1/298.257223563; */
     // R = 6378137; // m
     // Earth polar radius b = R * (1-f)
     // b = 6356752.31424518;
 
     Smu = sin(lat);
     Cmu = cos(lon);
     Sl  = sin(alt);
     Cl  = cos(lon);
     ECEF conv;
    
     // Radius of curvature at a surface point:
     Rn = REQ / sqrt(1.0 - E_SQ * Smu * Smu);
 
    conv.X=(Rn + alt) * Cmu * Cl;
    conv.Y=(Rn + alt) * Cmu * Sl;
    conv.Z=(Rn * POWB2 / POWA2 + alt) * Smu;
    return conv;
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

    bool floatinsteadofdouble=1;
    //print out the received lat
    // ROS_INFO("Latitude: [%f]", msg->latitude); 
    float lat = msg->latitude;
    float lon = msg->longitude;
    float alt = msg->altitude;
    ECEF newPos;
    // gps to ECEF
    if (floatinsteadofdouble){
        newPos = gpsToECEF(lat, lon, alt);
    }
    else
    {   
        newPos = llaToECEF(lat, lon, alt);
    }

    
    if (*toInit==true) {
        ROS_INFO("Initializing Datum");
        nh.setParam("/lat_zero", lat);
        nh.setParam("/lon_zero", lon);
        nh.setParam("/alt_zero", alt);
        if (floatinsteadofdouble){ *initFix = gpsToECEF(lat, lon, alt); }
        else {*initFix = llaToECEF(lat, lon, alt); }
        *toInit = false;
    } else {
        ROS_INFO("\nDelta in ECEF");
        ROS_INFO("Init X: [%f]", initFix->X); 
        ROS_INFO("DeltaX: [%f]", newPos.X-(initFix->X)); 
        ROS_INFO("DeltaY: [%f]", newPos.Y-(initFix->Y)); 
        ROS_INFO("DeltaZ: [%f]", newPos.Z-(initFix->Z)); 
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
    ROS_INFO("NED coordinates"); 
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
