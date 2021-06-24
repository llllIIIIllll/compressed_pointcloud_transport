#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

//PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/compression/octree_pointcloud_compression.h>

#include "compressed_pointcloud_interfaces/msg/compressed_point_cloud.hpp"


using namespace std::chrono_literals;

typedef pcl::PointXYZ PointT;


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal")
    {
        bool showStatistics = true;
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT> (compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT> ();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/uncompress", 10);
        
        subscription_ = this->create_subscription<compressed_pointcloud_interfaces::msg::CompressedPointCloud>(
        "/compress", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));


    }

    pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;


  private:
    void topic_callback(const compressed_pointcloud_interfaces::msg::CompressedPointCloud::SharedPtr msg) const
    {
        sensor_msgs::msg::PointCloud2::Ptr ros_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

        std::cout << "start" << msg->data.size() << std::endl;
        std::stringstream compressedData(msg->data);

        // Pack into a compressed message
        try
        {
          /* code */
          PointCloudDecoder->decodePointCloud(compressedData, pclCloud);
        }
        catch(const std::exception& e)
        {
          std::cerr << e.what() << '\n';
        }
        
        std::cout << "end" << std::endl;

        pcl::toROSMsg(*pclCloud, *ros_cloud);

        ros_cloud->header = msg->header;
        ros_cloud->header.frame_id = "front";

        publisher_->publish(*ros_cloud);

    }
    rclcpp::Subscription<compressed_pointcloud_interfaces::msg::CompressedPointCloud>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
