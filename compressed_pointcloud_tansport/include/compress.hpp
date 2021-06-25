#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

//PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/compression/octree_pointcloud_compression.h>

#include "compressed_pointcloud_interfaces/msg/compressed_point_cloud.hpp"


using namespace std::chrono_literals;

typedef pcl::PointXYZ PointT;

namespace compress_pt
{

class Compress : public rclcpp::Node
{
public:

    Compress();
	~Compress();

private:

    pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<compressed_pointcloud_interfaces::msg::CompressedPointCloud>::SharedPtr publisher_;

    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const ;

};

}; // namespace comporess_pt