#include "ros/ros.h"
#include "service/AddTwoInts.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  
  // we define our client to interact with the service, it expects our custom type AddTwoInts
  ros::ServiceClient client = n.serviceClient<service::AddTwoInts>("add_two_ints");
  service::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv)) // call our adding service, 
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum); //if the call is succesfull print result
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints"); //if the call fail print error
    return 1;
  }

  return 0;
}
