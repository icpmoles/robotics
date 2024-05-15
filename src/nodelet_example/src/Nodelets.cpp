#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <pluginlib/class_list_macros.h>

namespace nodelet_example {

class ProducerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        //ros::NodeHandle& private_nh = getPrivateNodeHandle();
        ros::NodeHandle& private_nh = getNodeHandle();
        publisher_ = private_nh.advertise<std_msgs::Int32>("output_topic", 10);
        timer_ = private_nh.createTimer(ros::Duration(1.0), &ProducerNodelet::timerCallback, this);
    }

    void timerCallback(const ros::TimerEvent&)
    {
        std_msgs::Int32 msg;
        msg.data = rand() % 100;  // Random number as data
        publisher_.publish(msg);
        NODELET_DEBUG("Produced: %d", msg.data);
    }

private:
    ros::Publisher publisher_;
    ros::Timer timer_;
};

class ConsumerNodelet : public nodelet::Nodelet
{
public:
    virtual void onInit()
    {
        ros::NodeHandle& nh = getNodeHandle();
        subscriber_ = nh.subscribe("output_topic", 10, &ConsumerNodelet::callback, this);
    }

    void callback(const std_msgs::Int32::ConstPtr& msg)
    {
        NODELET_INFO_STREAM("Received " << msg->data);
        ROS_INFO("Hey");
    }

private:
    ros::Subscriber subscriber_;
};

} // namespace nodelet_example

PLUGINLIB_EXPORT_CLASS(nodelet_example::ProducerNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(nodelet_example::ConsumerNodelet, nodelet::Nodelet)
