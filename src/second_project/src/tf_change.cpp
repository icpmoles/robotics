#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>

std::string newtf = "wheel_odom";


void filterCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    ros::Publisher pb){
        // copy packet and override header
        sensor_msgs::PointCloud2 newLS=*msg;
        newLS.header.frame_id = newtf;
        newLS.header.stamp = ros::Time::now();
        pb.publish(newLS);

}

int main(int argc, char** argv){

  ros::init(argc, argv, "lidar_remap");
  
  ros::NodeHandle nh;

  // we get the parameters of the topic to subscribe to
  ros::NodeHandle nh_private("~"); // this is a private node handle
 
  nh_private.getParam("root_frame", newtf);
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 2);
 
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("ugv/rslidar_points", 2,boost::bind(filterCallBack, _1, laser_pub));
  
  ros::spin();
  return 0;
};
