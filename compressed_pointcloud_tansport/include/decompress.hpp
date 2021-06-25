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

class DeCompress : public rclcpp::Node
{
public:

    DeCompress();
	~DeCompress();

private:

    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;

    rclcpp::Subscription<compressed_pointcloud_interfaces::msg::CompressedPointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    void topic_callback(const compressed_pointcloud_interfaces::msg::CompressedPointCloud::SharedPtr msg) const ;

};

}; // namespace comporess_pt