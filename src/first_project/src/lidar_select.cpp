#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <first_project/dynparametersConfig.h>
#include <sstream>

std::string TFframe = "gps_odom";

void paramCallback(first_project::dynparametersConfig &config, uint32_t level) {

  if (config.frame==0)
      TFframe = "gps_odom";
  else 
      TFframe = "odom";
 
  // ROS_INFO("Reconfigure Request:%s", 
  //           config.bool_param?"True":"False");
}

void filterCallBack(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    ros::Publisher pb){
        // copy packet and override header
        sensor_msgs::PointCloud2 newLS=*msg;
        newLS.header.frame_id = TFframe;
        newLS.header.stamp = ros::Time::now();
        pb.publish(newLS);

}

int main(int argc, char** argv){

  ros::init(argc, argv, "lidar_remap");
  
  ros::NodeHandle nh;

  // we get the parameters of the topic to subscribe to
  ros::NodeHandle nh_private("~"); // this is a private node handle
  std::string root_f, child_f;
  ros::Publisher laser_pub = nh.advertise<sensor_msgs::PointCloud2>("/pointcloud_remapped", 5);
  // dyn conf handles
  dynamic_reconfigure::Server<first_project::dynparametersConfig> server;
  dynamic_reconfigure::Server<first_project::dynparametersConfig>::CallbackType f;

  f = boost::bind(&paramCallback, _1, _2);
  server.setCallback(f);


  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("os_cloud_node/points", 10,boost::bind(filterCallBack, _1, laser_pub));
  
  ros::spin();
  return 0;
};
