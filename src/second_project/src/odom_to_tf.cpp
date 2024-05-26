#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string child_frame;

int i = 0;

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg,
                  std::string rf, std::string cf ) {

    static tf::TransformBroadcaster br;
    
    tf::Transform odomtobasefootprint;
    tf::Transform basefootprinttobaselink;
    tf::Transform baselinklaser;

  //   tf::Transform baseframetoodometry;
  //   tf::StampedTransform [3]
    geometry_msgs::Point cart_position =  msg->pose.pose.position;
    geometry_msgs::Quaternion cart_heading  = msg->pose.pose.orientation;
    odomtobasefootprint.setOrigin( tf::Vector3(cart_position.x, cart_position.y, cart_position.z) );
    tf::Quaternion q;
    q.setW(cart_heading.w);
    q.setX(cart_heading.x);
    q.setY(cart_heading.y);
    q.setZ(cart_heading.z);
    q.normalize();
    odomtobasefootprint.setRotation(q); 
  //   basetolasertransform.setOrigin( tf::Vector3(0.0, 0.0, 0.03) );  // height of robot is 245mm->24.5mm->0.0245m
  //   basetolasertransform.setRotation( tf::Quaternion(0, 0, 0, 1) );
  //   br.sendTransform(tf::StampedTransform(basetolasertransform, ros::Time::now(), "rslidar", "base_footprint"));

    

    basefootprinttobaselink.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );   //attached links for our vehicle
    basefootprinttobaselink.setRotation( tf::Quaternion(0, 0, 0, 1) );



    baselinklaser.setOrigin( tf::Vector3(0.0, 0.0, 0.245) );   //attached links for our vehicle
    basefootprinttobaselink.setRotation( tf::Quaternion(0, 0, 0, 1) );

    
    // br.sendTransform(tf::StampedTransform(odomtobasefootprint, ros::Time::now(), rf, cf));
    // br.sendTransform(tf::StampedTransform(basefootprinttobaselink, ros::Time::now(), "base_footprint", "base_link"));
    // br.sendTransform(tf::StampedTransform(basefootprinttobaselink, ros::Time::now(), "base_link", "rslidar"));

    if (i == 0)
    {br.sendTransform({
    tf::StampedTransform(odomtobasefootprint, ros::Time::now(), rf, cf),
    tf::StampedTransform(basefootprinttobaselink, ros::Time::now(), "base_footprint", "base_link"),
    tf::StampedTransform(basefootprinttobaselink, ros::Time::now(), "base_link", "rslidar")
    });
    i++;}
    else 
    br.sendTransform(tf::StampedTransform(odomtobasefootprint, ros::Time::now(), rf, cf));

    
}

int main(int argc, char** argv){
  //child_frame = argv[1];

  ros::init(argc, argv, "o2tf_", 
            ros::init_options::AnonymousName);
  
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  // we get the parameters of the topic to subscribe to
	ros::NodeHandle nh_private("~"); // this is a private node handle
  std::string topic, root_f, child_f;
	// nh_private.getParam("input_odom", topic); //get local params
  nh_private.getParam("root_frame", root_f);
  nh_private.getParam("child_frame", child_f);
  topic = "input_odom";

  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic, 2,boost::bind(poseCallback, _1, root_f, child_f));
  
  ros::spin();
  return 0;
};