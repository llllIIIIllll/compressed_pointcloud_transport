#include "decompress.hpp"

namespace compress_pt
{
    DeCompress::DeCompress()
    : Node("minimal")
    {
        PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT> ();

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/uncompress", 10);
        
        subscription_ = this->create_subscription<compressed_pointcloud_interfaces::msg::CompressedPointCloud>(
        "/compress", 10, std::bind(&DeCompress::topic_callback, this, std::placeholders::_1));

    }

    DeCompress::~DeCompress()
    {}

    void DeCompress::topic_callback(const compressed_pointcloud_interfaces::msg::CompressedPointCloud::SharedPtr msg) const
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

};

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<compress_pt::DeCompress>());
  rclcpp::shutdown();
  return 0;
}
