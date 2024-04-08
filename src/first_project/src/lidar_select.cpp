#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
// #include <parameter_test/parametersConfig.h>
#include <sensor_msgs/LaserScan.h>



void filterCallBack(
    const sensor_msgs::LaserScan::ConstPtr& msg,
    ros::Publisher pb){
        // copy packet and override header
        sensor_msgs::LaserScan newLS=*msg;
        newLS.header.frame_id = "odom";
        newLS.header.stamp = ros::Time::now();
        pb.publish(newLS);

}

int main(int argc, char** argv){

  ros::init(argc, argv, "lidar_f");
  
  ros::NodeHandle nh;

  // we get the parameters of the topic to subscribe to
    ros::NodeHandle nh_private("~"); // this is a private node handle
    std::string root_f, child_f;
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("/laser", 5);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("scan", 10,boost::bind(filterCallBack, _1, laser_pub));
  
  ros::spin();
  return 0;
};
