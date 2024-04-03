#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf/transform_broadcaster.h>

class tf_sub_pub{
public:
  	tf_sub_pub(){
  	sub = n.subscribe("/turtle1/pose", 1000, &tf_sub_pub::callback, this);
}


void callback(const turtlesim::Pose::ConstPtr& msg){
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0) ); // set initial position of the transformation, nb: 2d so z=0
  tf::Quaternion q;  // rotation in tf are defined by quaternions
  q.setRPY(0, 0, msg->theta);  // Roll-Pitch-Yaw automatically as a quaternions
  transform.setRotation(q); //use our quaternion
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "turtle"));  // publish our transformation, n.b: Ros wants a transform with a timestamp
}

private:
  ros::NodeHandle n; 
  tf::TransformBroadcaster br;
  ros::Subscriber sub;
};


int main(int argc, char **argv){
 ros::init(argc, argv, "subscribe_and_publish");
 tf_sub_pub my_tf_sub_bub;
 ros::spin();
 return 0;
}

