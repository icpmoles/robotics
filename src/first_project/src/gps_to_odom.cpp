#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/TransformStamped.h>
#include <queue>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sstream>
#include <cmath>
#include <complex>
#include <iomanip>
#include <iostream>
#define PI 3.14159265358979323
// taken from https://en.wikipedia.org/wiki/Latitude#The_geometry_of_the_ellipsoid
#define A_EQRAD 6378137
#define B_POLRAD 6356752.31425
#define E_SQUARED 0.00669437999014
#define MA_SIZE 2 // moving average size



double heading_zero;


double RadToDeg(double radians){
    return radians*180.0/PI;
}

struct ECEF {
    double X, Y, Z;
};

struct ENU{
    double N = 0, E = 0, U = 0;
    ros::Time timestamp ;
    double Y = 0; //yaw, may be useful for angular speed 
};


std::deque<double> N_queue(MA_SIZE, 0.0); // first element = oldest, last = most recent
std::deque<double> E_queue(MA_SIZE, 0.0); // first element = oldest, last = most recent
std::deque<ENU> Pose_queue(MA_SIZE); // first element = oldest, last = most recent


// https://en.wikipedia.org/wiki/Circular_mean
double MovingAverage( std::deque<double> const &q){
    double sum = 0.0;
    // ROS_INFO("New MA");
   

    for (int i = 0; i < q.size(); i++) {

        sum += q[i];
        //  ROS_INFO("values: %f", q[i]);
    }
    return sum;
}

double SimpleEstimator(){

    return atan2( MovingAverage(N_queue), MovingAverage(E_queue)) ;

}
// given a sequence of poses, esimates the p_tilde coefficent
double PCoeff(ENU pose0, ENU pose1, ENU pose2 ){

    double x0=pose0.E;
    double x1=pose1.E;
    double x2=pose2.E;
    double y0=pose0.N;
    double y1=pose0.N;
    double y2=pose0.N;
    
    
    // phi_ab = atan2 (yb-ya,xb-xa)
    double phi_01=atan2( y1-y0 , x1-x0 ); //heading two steps back
    double phi_12=atan2( y2-y1 , x2-x1 ); //heading one step back
    

    return 0.5*( cos(phi_12)*(x0-x2) +  sin(phi_12)*(y0-y2) ) / ( sin(phi_01 - phi_12) );

}

double RadiusSlope(std::deque<ENU> const &pose ){
    // uses a circumference to calculate the predicted heading
    // see notes for understanding
    // we take t-2="0", t-1="1", t="2"   
    // z[sz-1]    is z at t
    // z[sz-1 -1] is z at t-1
    // z[sz-1 -2] is z at t-2
    int sz=pose.size();
    double x0=pose[sz-1 -2].N;
    double x1=pose[sz-1 -1].N;
    double x2=pose[sz-1   ].N;
    double y0=pose[sz-1 -2].E;
    double y1=pose[sz-1 -1].E;
    double y2=pose[sz-1   ].E;
    
    
    // phi_ab = atan2 (yb-ya,xb-xa)
    double phi_01=atan2( y1-y0 , x1-x0 ); //heading two steps back
    double phi_12=atan2( y2-y1 , x2-x1 ); //heading one step back
    

    double p = 0.5*( cos(phi_12)*(x0-x2) +  sin(phi_12)*(y0-y2) ) / ( sin(phi_01 - phi_12) );
    double CenterX=0.5*(x0+x1)-p*sin(phi_01);
    double CenterY=0.5*(y0+y1)+p*cos(phi_01);


    double theta_hat, theta =0.0;
    theta_hat=atan2(y2-CenterY,x2-CenterX);
    if (p>0) { 
        theta = theta_hat + PI/2;
     }
    else{
         theta = theta_hat - PI/2;
    }

    return theta;
}


// it takes the queue of x and y and decides the best algorithm to use
double SlopeCalculator(std::deque<ENU> const &pose, std::deque<double> const &N_queue,std::deque<double> const &E_queue){
    // measure the total heading change of the queue (in radians)
    double factor = 0.2;
    double averHCd, totHeadingChange = 0.0;
    int inaTurn = false;
    // for (int i = pose.size()-1; i > 4; i--) // if for 5 positions we turn on the same direction (CCw or CW) then we are in a turn an not just 
    // //in a straight line
    // {
    //     bool s0 = PCoeff(pose[i],pose[i-1],pose[i-2])>0;
    //     int s1 = (PCoeff(pose[i-1],pose[i-2],pose[i-3]))>0;
    //     int s2 = (PCoeff(pose[i-2],pose[i-3],pose[i-4]))>0;
    //     ROS_INFO("%i %i %i",s0,s1,s2);
    //     if ((s0==s1) and (s1==s2)){
    //     inaTurn = true;
    //     ROS_INFO("In a turn");
    //     }
    //     else{
    //     ROS_INFO("Straight");
    //     }
    // }
    

    double theta_p = SimpleEstimator(); // use the simple algorithm for a rough estimate (like a Proportional action)
    double theta_d_E = RadiusSlope(Pose_queue); // use a "derivative action" to predict ahead (?)
    
    std::complex<double> v_p = std::polar(1.0 , theta_p);  // versor of proportial action
    std::complex<double> v_d = std::polar(1.0 , theta_d_E); // versor of derivative action
    return std::arg(v_p + factor*v_d );
    // if (inaTurn){
    //     return RadiusSlope(Pose_queue);
    // }
    // else { // otherwise we are in a straight line and we can just use a circular moving average
        
    //     return atan2( MovingAverage(N_queue), MovingAverage(E_queue)) ;
    // }
   
}




double prime_vertical(double lat ){
    double a = A_EQRAD ; // equatorial radius in meters
    //float b = B_POLRAD; // polar radius in meters
    double e2 =  E_SQUARED;
    return a / sqrt(1-e2*pow(sin(lat),2));
}

// https://onlinelibrary.wiley.com/doi/pdf/10.1002/9780470099728.app3 C.80 -> C.82
// ECEF coordinates plus latitude and longitude (in radians) of the "landmark reference" as float, and the new position in ECEF
ENU ECEFtoENU(ECEF datum, double lat, double lon, ECEF position){ 
    double deltaX=position.X-datum.X;
    double deltaY=position.Y-datum.Y;
    double deltaZ=position.Z-datum.Z;

    ENU temp;
    temp.N =-cos(lon)* sin(lat)  *deltaX  
            -sin(lon)* sin(lat)  *deltaY 
                    +  cos(lat) *deltaZ;
    temp.E =   -sin(lon)*deltaX + 
                cos(lon)*deltaY;
    temp.U =-( -cos(lon)*cos(lat)*deltaX + 
            (-sin(lon)*cos(lat))*deltaY +
                    (-sin(lat))*deltaZ ); 
    return temp;           
}


// LLA in radians please
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


bool initialHeadingEstimated = false;

void gpsCallback(   const sensor_msgs::NavSatFix::ConstPtr& msg, 
                    bool *toInit, 
                    ros::NodeHandle nh, 
                    ros::Publisher ph,
                    ECEF *initFix,
                    ENU *prevPo)
{
    
    bool useENUframe = false;
    bool logging = false;
    //print out the received lat
    // ROS_INFO("Latitude: [%f]", msg->latitude); 
    // in deg as provided by the GPS fix then they are converted in radians
    double lat = msg->latitude *PI/180;
    double lon = msg->longitude*PI/180;
    double alt = msg->altitude *PI/180;
    // gps to ECEF
    ECEF newPos = gpsToECEF(lat, lon, alt);
    bool new_algo = false; // i tried implementing a more sophisticated algo, it didn't work. keep it false
    
    if (*toInit==true) {
        ROS_INFO("Initializing Datum");
        nh.setParam("/lat_zero", msg->latitude);
        nh.setParam("/lon_zero", msg->longitude);
        nh.setParam("/alt_zero", msg->altitude);
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
    data.header.frame_id= "world";
    data.child_frame_id = "gps_transceiver";
    // ECEF to NED
    ENU actualENU = ECEFtoENU(*initFix,lat,lon,newPos); // NED at t
    actualENU.timestamp=msg->header.stamp; // the NED positione was "acquired" at the same time of the Fix
   


    //calculate heading
    double delta_space=0; //distance between two fixes
    double yaw_est = 0; // yaw of vehicle fixed to E axis, positive ccw
    double yaw_est_simple = 0;
    double prevYaw;
    double yaw_deriv = 0; //velocity of rotation in the yaw axis, just for curiosity
    double delta_N=0; // difference in 2d NED place
    double delta_E=0;
    ros::Duration delta_T(0.5);
    double diff_t;
    double vel;
    tf::Quaternion q; 
    // if we don't have an initial heading estimation and we just got out
    // from a circle of radius 0.3m we don't initialize the initialHeading
    // 
    if ( (!initialHeadingEstimated) && (sqrt(actualENU.N*actualENU.N + actualENU.E*actualENU.E) > 0.3) ) { 
        initialHeadingEstimated = true;
        heading_zero = atan2(actualENU.N,actualENU.E);
    }

    // if (msg->header.seq>1)  {// for a bug that I forgot about
    
    if ( abs((actualENU.timestamp - prevPo->timestamp).toSec()) < 5 )
    { 
        //in case the delta_T is "reasonable" we use it instead of the deafult 0.5s
        delta_T = actualENU.timestamp - prevPo->timestamp;
    }
    
    diff_t = delta_T.toSec();
    delta_N = actualENU.N - prevPo->N;
    delta_E = actualENU.E - prevPo->E;
    delta_space = sqrt( pow(delta_N,2) + pow(delta_E,2) );
    vel = delta_space/diff_t;

    Pose_queue.push_back(actualENU);
    
    // fill our queue
    N_queue.push_back(delta_N);
    E_queue.push_back(delta_E);
    
    // if we don't move much in the refresh window it's hard to estimate the heading becuase of the uncertinty of the GPS, 
    // so we just use the previous heading and stick with it
    if (initialHeadingEstimated) { //(delta_space>0.01){ 
        
        // calculate MovingAverage
        // yaw_est = atan2( MovingAverage(N_queue), MovingAverage(E_queue)) ; // - heading_zero;
       
       if (new_algo) 
       yaw_est = SlopeCalculator(Pose_queue,E_queue, N_queue);
       else
       yaw_est=SimpleEstimator();


        yaw_est_simple = atan2(delta_N,delta_E) - heading_zero;
    }
    else { // if we are not at the start we can use this trick otherwise it makes up angles
        yaw_est = prevPo->Y;
        yaw_est_simple = prevPo->Y;
    }

    // pop old values
    N_queue.pop_front();
    E_queue.pop_front();
    Pose_queue.pop_front();
        
    if (useENUframe){ 
    // NED to XYZ ?????? ENU https://www.ros.org/reps/rep-0103.html
    data.pose.pose.position.y=actualENU.N; 
    data.pose.pose.position.x=actualENU.E;
    data.pose.pose.position.z=actualENU.U;
    q.setRPY( 0, 0, yaw_est);
    data.pose.pose.orientation.w = q.getW();
    data.pose.pose.orientation.x = q.getX();
    data.pose.pose.orientation.y = q.getY();
    data.pose.pose.orientation.z = q.getZ();

    }
    else { //debias
        std::complex<double> v_p = std::polar(1.0 , yaw_est -heading_zero) ;  //versor of direction in polar coordinates aligned to staring frame
        double r = sqrt( pow(actualENU.N,2) + pow(actualENU.E,2) );
        std::complex<double> p_p = std::polar( r , atan2(actualENU.N,actualENU.E) - heading_zero) ; // position in the new frame


        data.pose.pose.position.x=std::real(p_p);
        data.pose.pose.position.y=std::imag(p_p); 
        data.pose.pose.position.z=actualENU.U;

        
        q.setRPY( 0, 0, std::arg(v_p));
        data.pose.pose.orientation.w = q.getW();
        data.pose.pose.orientation.x = q.getX();
        data.pose.pose.orientation.y = q.getY();
        data.pose.pose.orientation.z = q.getZ();



    }


    actualENU.Y = yaw_est;
    yaw_deriv= (yaw_est - prevPo->Y)/diff_t;





    
    



   
   // }  otherwise we simply update 

    *prevPo = actualENU;
    
    

    data.twist.twist.linear.x=vel;
    data.twist.twist.angular.z=heading_zero;
    data.twist.twist.angular.y=yaw_est -heading_zero; //yaw_est_simple; // for debugging ; 
   


    if (logging){
    data.twist.twist.linear.y=diff_t;
     ROS_INFO("DeltaT is %f", diff_t);    
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

    ROS_INFO("ENU coordinates"); 
    ROS_INFO("DeltaN: [%f]", actualENU.N); 
    ROS_INFO("DeltaE: [%f]", actualENU.E); 
    ROS_INFO("DeltaU: [%f]", actualENU.U);
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
    ENU prevPoistion;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("gps_odom", 1);
    // subscribe to gps data
    // ros::Subscriber gps_sub = nh.subscribe("fix", 1, gpsCallback);
    // the callback functions also gets the bool if it's supposed to set the datum or not
    ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix> ("fix", 2, boost::bind(gpsCallback, _1, &datumToInit, nh, odom_pub,&initialECEF, &prevPoistion));
    // datumToInit=false; //we assume that the callback is going to do its job somehow

	ros::Rate loop_rate(8); 
    

  	while (ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();	
  	}
  	return 0;
}
