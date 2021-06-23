#include <chrono>
#include <functional>
#include <memory>
#include <string>

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


class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
        bool showStatistics = true;
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT> (compressionProfile, showStatistics);
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT> ();

        publisher_ = this->create_publisher<compressed_pointcloud_interfaces::msg::CompressedPointCloud>("/compress", 10);
        publisher_output = this->create_publisher<sensor_msgs::msg::PointCloud2>("/uncompress", 10);
        
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/id/pandar/front", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));


    }

    pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;


  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);
        pcl::PointCloud<PointT>::Ptr pclCloud2(new pcl::PointCloud<PointT>);
        sensor_msgs::msg::PointCloud2::Ptr ros_cloud(new sensor_msgs::msg::PointCloud2);

        // Stringstream to store compressed point cloud
        std::cout << "start" << std::endl;
        std::stringstream compressedData;
        // Must convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointT> for the encoder
        pcl::fromROSMsg(*msg, *pclCloud);

        // Compress the pointcloud
        PointCloudEncoder->encodePointCloud (pclCloud, compressedData);

        // Pack into a compressed message
        compressed_pointcloud_interfaces::msg::CompressedPointCloud output;
        output.header = msg->header;
        output.data = compressedData.str();
        publisher_->publish(output);

       PointCloudDecoder->decodePointCloud(compressedData, pclCloud2);
 
       pcl::toROSMsg(*pclCloud2, *ros_cloud);
       ros_cloud->header = msg->header;

      publisher_output->publish(*ros_cloud);
        std::cout << "end" << std::endl;

    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<compressed_pointcloud_interfaces::msg::CompressedPointCloud>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_output;

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
