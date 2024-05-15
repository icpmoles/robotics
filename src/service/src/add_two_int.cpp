#include "ros/ros.h"
#include "service/AddTwoInts.h"

//callback function for our service
bool add(service::AddTwoInts::Request  &req, // type taken by AddTwoInts.srv
         service::AddTwoInts::Response &res)  
{
  res.sum = req.a + req.b;  // from AddTwoInts.srv
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true; // to publish the response we simply complete the fields and return true
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
