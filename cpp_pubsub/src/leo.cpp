#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// #include <sensor_msgs/PointCloud2.h>

#include <Eigen/Core>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/search/impl/search.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>

using namespace std::chrono_literals;

class PointCloudProcessor
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointSource = pcl::PointXYZ;
    using PointTarget = pcl::PointXYZ;
    using PointT = pcl::PointXYZ;

    double score;
    bool is_converged;

    // PointCloud::Ptr cloud1;
    // PointCloud::Ptr cloud2;

    Eigen::Matrix4f transform_finder(PointCloud pcl_cloud1, PointCloud pcl_cloud2)
    // Find transformation between 2 point clouds
    {
        pcl::registration::TransformationEstimationSVD<PointSource, PointTarget> TESVD;
        pcl::registration::TransformationEstimation<PointSource, PointTarget>::Matrix4 trans_matrix;

        TESVD.estimateRigidTransformation(pcl_cloud1, pcl_cloud2, trans_matrix);

        // RCLCPP_INFO_STREAM(this->get_logger(), "Transformation matrix from my function: \n"
        //           << "    | %6.3f %6.3f %6.3f | \n" << trans_matrix (0,0) <<  trans_matrix (0,1) <<  trans_matrix (0,2)
        //           << "R = | %6.3f %6.3f %6.3f | \n" <<  trans_matrix (1,0) <<  trans_matrix (1,1) <<  trans_matrix (1,2)
        //           << "    | %6.3f %6.3f %6.3f | \n" <<  trans_matrix (2,0) <<  trans_matrix (2,1) <<  trans_matrix (2,2)
        //           << "\n"
        //           << "t = < %0.3f, %0.3f, %0.3f >\n" <<  trans_matrix (0,3) <<  trans_matrix (1,3) <<  trans_matrix (2,3)
        //           );

        // std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
        // printf ("\n");
        // printf ("    | %6.3f %6.3f %6.3f | \n", trans_matrix (0,0), trans_matrix (0,1), trans_matrix (0,2));
        // printf ("R = | %6.3f %6.3f %6.3f | \n", trans_matrix (1,0), trans_matrix (1,1), trans_matrix (1,2));
        // printf ("    | %6.3f %6.3f %6.3f | \n", trans_matrix (2,0), trans_matrix (2,1), trans_matrix (2,2));
        // printf ("\n");
        // printf ("t = < %0.3f, %0.3f, %0.3f >\n", trans_matrix (0,3), trans_matrix (1,3), trans_matrix (2,3));

        return trans_matrix;
    };

    // pcl::GeneralizedIterativeClosestPoint<PointSource, PointTarget> gicp;

    PointCloudProcessor()
    {
    }
};

class MainClass : public rclcpp::Node
{
public:
    std::string pcd_file1;
    std::string pcd_file2;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_publisher_2;

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointSource = pcl::PointXYZ;
    using PointTarget = pcl::PointXYZ;
    using PointT = pcl::PointXYZ;

    PointCloudProcessor pc_processor;

    



    pcl::PointCloud<pcl::PointXYZ>::Ptr file_loader(std::string file)
    // Loads a .pcd file and returns a cloud
    {
        this->declare_parameter<std::string>(file, "default"); //"pcd_file1"

        this->get_parameter(file, pcd_file1); //"pcd_file1"

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        pcl::io::loadPCDFile<pcl::PointXYZ>(this->pcd_file1, *cloud);

        return cloud;
    };

    MainClass()
        : Node("point_cloud_registering")
    {
        // publishers

        pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points1", 10);
        pc_publisher_2 = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points2", 10);

        // publish on ros2

        sensor_msgs::msg::PointCloud2 ros_cloud1;
        sensor_msgs::msg::PointCloud2 ros_cloud2;

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud1 = file_loader("pcd_file1");
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud2 = file_loader("pcd_file2");

        pcl::toROSMsg(*pcl_cloud1, ros_cloud1);
        pcl::toROSMsg(*pcl_cloud2, ros_cloud2);
        ros_cloud1.header.frame_id = "map";
        ros_cloud1.header.stamp = this->get_clock()->now();

        ros_cloud2.header.frame_id = "map";
        ros_cloud2.header.stamp = this->get_clock()->now();

        RCLCPP_INFO_STREAM(this->get_logger(), "Loaded "
                                                   << pcl_cloud1->width * pcl_cloud1->height
                                                   << " data points from .pcd with the following fields: ");

        pc_publisher_->publish(ros_cloud1);
        pc_publisher_2->publish(ros_cloud2);

        // gicp part

        //pcl::PointCloud<PointSource>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::PointCloud<PointSource>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);

        

        // cloud1 = pcl_cloud1;
        // cloud2 = pcl_cloud2;

        //   Eigen::Matrix<float, 4, 4, 0, 4, 4> trans_matrix;

        //    trans_matrix <<   1, 0, 0, 2.0,
        //                      0, 1, 0, 0.0,
        //                      0, 0, 1, 0.0,
        //                      0, 0, 0, 1.0;

        // pcl::transformPointCloud(*cloud1, *cloud2, trans_matrix);

         

        // gicp.setMaxCorrespondenceDistance(1.0);
        // gicp.setTransformationEpsilon(0.001);
        // gicp.setMaximumIterations(1000);

        // // pcl::PointCloud<pcl::PointXYZ> aligned_cloud;

        //    gicp.setInputSource(cloud1);
        //    gicp.setInputTarget(cloud2);
        // // gicp.align(aligned_cloud);

        //    Eigen::Matrix4f src2tgt   = gicp.getFinalTransformation();
        //    double score              = gicp.getFitnessScore();
        //    bool is_converged         = gicp.hasConverged();

        // int x = 0;
        // while(true){
        //   if(x % 8000000 == 0) {
        //     publisher_->publish(ros_cloud1);
        //     publisher_2->publish(ros_cloud2);
        //     //RCLCPP_INFO(this->get_logger(), "published %d", x);
        //     }
        //   x++;
        // }
    };
};

// using std::placeholders::_1;

// class StaticFramePublisher : public rclcpp::Node
// {
// public:
//   explicit StaticFramePublisher(const char * transformation[])
//   : Node("transform_node")
//   {
//     rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_publisher_;
//     rclcpp::Time now = this->get_clock()->now();
//     geometry_msgs::msg::TransformStamped world_t;
//     world_t.header.stamp = now;
//     world_t.header.frame_id = "world";
//     world_t.child_frame_id = "";
//     world_t.transform.translation.x = 0;
//     world_t.transform.translation.x = 0;
//     world_t.transform.translation.x = 0;
//     world_t.transform.rotation.x = 0;
//     world_t.transform.rotation.y = 0;
//     world_t.transform.rotation.z = 0;
//     world_t.transform.rotation.w = 0;
//     //tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
//     tf_publisher_->publish(world_t);
//     RCLCPP_INFO(this->get_logger(), "published");

//     // Publish static transforms once at startup
//     this->make_transforms(transformation);
//   }

// private:
//   void make_transforms(const char * transformation[])
//   {
//     geometry_msgs::msg::TransformStamped t;
//     rclcpp::Time now = this->get_clock()->now();

//     t.header.stamp = now;
//     t.header.frame_id = "world";
//     t.child_frame_id = transformation[0];

//     t.transform.translation.x = atof(transformation[1]);
//     t.transform.translation.y = atof(transformation[2]);
//     t.transform.translation.z = atof(transformation[3]);
//     tf2::Quaternion q;
//     q.setRPY(
//       atof(transformation[4]),
//       atof(transformation[5]),
//       atof(transformation[6]));
//     t.transform.rotation.x = q.x();
//     t.transform.rotation.y = q.y();
//     t.transform.rotation.z = q.z();
//     t.transform.rotation.w = q.w();

//     tf_publisher_->sendTransform(t);
//   }
//   std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
// };

int main(int argc, char **argv)
{

    // child_frame_name x y z roll pitch yaw
    // const char *transformation1[7] = {"point_cloud_1", "0", "0", "0", "0", "0", "0"};
    // const char *transformation2[7] = {"point_cloud_2", "0", "0", "0", "0", "0", "0"};

    //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MainClass>());

    rclcpp::shutdown();

    // RCLCPP_INFO(param.get_logger(), "bittim");

    return (0);
}