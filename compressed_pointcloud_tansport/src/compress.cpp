
#include "compress.hpp"

namespace compress_pt
{
    Compress::Compress()
    : Node("minimal_subscriber")
    {
        this->get_parameter_or("input_topic_name", input_topic_name_, std::string("/id/pandar/front"));
        this->get_parameter_or("output_topic_name", output_topic_name_, std::string(input_topic_name_ + "/compress"));

        bool showStatistics = true;
        pcl::io::compression_Profiles_e compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

        PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT> (compressionProfile, showStatistics);

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic_name_, 10, std::bind(&Compress::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<compressed_pointcloud_interfaces::msg::CompressedPointCloud>(output_topic_name_, 10);
    }

    Compress::~Compress()
    {}

    void Compress::topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
        pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

        // Stringstream to store compressed point cloud
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
    }

}; // namespace comporess_pt

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<compress_pt::Compress>());
  rclcpp::shutdown();
  return 0;
}