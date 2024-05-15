#include <ros/ros.h>
#include <std_msgs/String.h>
void callbackTalker1(const std_msgs::String::ConstPtr &msg)
{

    ros::Duration(2.0).sleep(); // simulate long processing
    ROS_INFO_STREAM("Message from callback 1:" );
    
    ROS_INFO("%s", msg->data.c_str());
}

void callbackTalker2(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO_STREAM("Message from callback 2:");
    ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker_subscribers_asynch");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(0); // num of thread. 0-> max of thread as possible
    spinner.start();  // starts the spinner, easy,  only change for asynch stuff
    
    ros::Subscriber counter1_sub = nh.subscribe("talker1", 1, callbackTalker1);
    ros::Subscriber counter2_sub = nh.subscribe("talker2", 1, callbackTalker2);
    ros::waitForShutdown();
}
