#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
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

#include <Eigen/Core>
#include <kml/base/file.h>
#include <kml/engine.h>
#include <kml/dom.h>


typedef std::vector<kmldom::LineStringPtr> LineStringVector;


class KmlClass : public rclcpp::Node
{
public:
    const char* kmlfile;
    std::string file_data;
    kmlbase::File kmlbase;
    std::string kml;
    std::string errors;

    kmlengine::KmlFilePtr kml_file;
    kmldom::FeaturePtr root;
    kmldom::KmlDomType type;

    LineStringVector line_string_vector;
    
    
    KmlClass()
        : Node("kml_parser")
    {
        // // check if file can be read

        // if (!kmlbase::File::ReadFileToString(kmlfile, &file_data)) {
        //     std::cout << kmlfile << " read failed" << std::endl;
        // }

        // // read the file if file can be read

        // if (kmlengine::KmzFile::IsKmz(file_data)) {
        //     kmz_file = kmlengine::KmzFile::OpenFromString(kmlfile);
        //     if (!kmz_file) {
        //     std::cout << "Failed opening KMZ file" << std::endl;
        //     }
        //     if (!kmz_file->ReadKml(&kml)) {
        //     std::cout << "Failed to read KML from KMZ" << std::endl;
        //     }
        // } else {
        //     kml = file_data;
        // }

        // get the file
        kml_file = kmlengine::KmlFile::CreateFromParse(kml, &errors);

        // read the file features
        root = kmlengine::GetRootFeature(kml_file->get_root());

        // get the root type
        type = root->Type();
        RCLCPP_INFO_STREAM(this->get_logger(), "root type: " << type);




        while (rclcpp::ok())
        {
            
        }

    };
};




int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KmlClass>());

    rclcpp::shutdown();

    return (0);
}