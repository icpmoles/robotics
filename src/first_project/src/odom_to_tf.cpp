#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

std::string child_frame;



void poseCallback(
    const nav_msgs::Odometry::ConstPtr& msg,
    std::string rf, std::string cf ){

    static tf::TransformBroadcaster br;
    tf::Transform transform;
  
    geometry_msgs::Point cart_position =  msg->pose.pose.position;
    geometry_msgs::Quaternion cart_heading  = msg->pose.pose.orientation;
    transform.setOrigin( tf::Vector3(cart_position.x, cart_position.y, 0.0) );
    tf::Quaternion q;
    q.setW(cart_heading.w);
    q.setX(cart_heading.x);
    q.setY(cart_heading.y);
    q.setZ(cart_heading.z);
    q.normalize();
    transform.setRotation(q);


    if ((cart_position.x<=-0.001302 and cart_position.x>=-0.001303) or (cart_position.y<=0.000371 and cart_position.y>=0.000369) )
    {
            
        ROS_INFO("Did it crash?");    
    }

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), rf, cf));
}

int main(int argc, char** argv){
  child_frame = argv[1];

  ros::init(argc, argv, "o2tf_"+child_frame, 
            ros::init_options::AnonymousName);
  
  ros::NodeHandle nh;
  tf::TransformBroadcaster br;

  // we get the parameters of the topic to subscribe to
	ros::NodeHandle nh_private("~"); // this is a private node handle
  std::string topic, root_f, child_f;
	nh_private.getParam("input_odom", topic); //get local params
  nh_private.getParam("root_frame", root_f);
  nh_private.getParam("child_frame", child_f);


  ros::Subscriber sub = nh.subscribe<nav_msgs::Odometry>(topic, 10,boost::bind(poseCallback, _1, root_f, child_f));
  
  ros::spin();
  return 0;
};