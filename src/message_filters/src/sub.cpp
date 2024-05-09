#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
// mesage filter libs
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>



void callback(const geometry_msgs::Vector3StampedConstPtr& msg1, const geometry_msgs::Vector3StampedConstPtr& msg2)
{
  ROS_INFO ("Received two messages: (%f,%f,%f) and (%f,%f,%f)", msg1->vector.x,msg1->vector.y,msg1->vector.z, msg2->vector.x, msg2->vector.y, msg2->vector.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "subscriber");

  ros::NodeHandle n;


// we create subscribers, little different than normal: message_filters subscriber
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub1(n, "topic1", 1);
  message_filters::Subscriber<geometry_msgs::Vector3Stamped> sub2(n, "topic2", 1);

  // we create time synchronizer
  // <type first message, type second message> sync (mfsubscriber 1 , mf subscriber 2, queue size)
  message_filters::TimeSynchronizer<geometry_msgs::Vector3Stamped, geometry_msgs::Vector3Stamped> sync(sub1, sub2, 10);

  // register &callback to synchronizer with 2 placeholders
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}


