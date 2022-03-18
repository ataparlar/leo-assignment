#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// #include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

using namespace std::chrono_literals;


class ParametersClass: public rclcpp::Node
{
  public:
    std::string pcd_file1;
    std::string pcd_file2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_2;
    
    ParametersClass()
     : Node("point_cloud_registering")
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points1", 10);
      publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10);
      this->declare_parameter<std::string>("pcd_file1", "default");
      this->declare_parameter<std::string>("pcd_file2", "default");

      this->get_parameter("pcd_file1", pcd_file1);
      this->get_parameter("pcd_file2", pcd_file2);


      sensor_msgs::msg::PointCloud2 ros_cloud1;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1 (new pcl::PointCloud<pcl::PointXYZ>);
      
      sensor_msgs::msg::PointCloud2 ros_cloud2;
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
      
      pcl::io::loadPCDFile<pcl::PointXYZ> (this->pcd_file1, *pcl_cloud1);
      pcl::io::loadPCDFile<pcl::PointXYZ> (this->pcd_file2, *pcl_cloud2);

      pcl::toROSMsg(*pcl_cloud1, ros_cloud1);
      pcl::toROSMsg(*pcl_cloud2, ros_cloud2);
      ros_cloud1.header.frame_id = "map";
      ros_cloud1.header.stamp = this->get_clock()->now();

      ros_cloud2.header.frame_id = "map";
      ros_cloud2.header.stamp = this->get_clock()->now();

      RCLCPP_INFO_STREAM(this->get_logger(), "Loaded "
                << pcl_cloud1->width * pcl_cloud1->height
                << " data points from .pcd with the following fields: ");
                
                

      int x = 0;
      while(true){
        if(x % 8000000 == 0) {
          publisher_->publish(ros_cloud1);
          publisher_2->publish(ros_cloud2);
          RCLCPP_INFO(this->get_logger(), "published %d", x);
          }
        x++;
      }
    }      
};



// class PointCloudPublisherClass: public rclcpp::Node
// {
//   sensor_msgs::PointCloud2 points1;
//   sensor_msgs::PointCloud2 points2;

  

//   public:
//     PointCloudPublisherClass()
//     : Node("pc_publisher")
//     {
      
//       //publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("points1", 10);

//     }
// };


int main (int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ParametersClass>());
  rclcpp::shutdown();


  // RCLCPP_INFO(param.get_logger(), "bittim");  

  return (0);
}