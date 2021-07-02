#include "decompress.hpp"

namespace compress_pt
{
    DeCompress::DeCompress()
    : Node("decompress", 
        rclcpp::NodeOptions(
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
        )
    )
    {
        this->get_parameter_or("input_topic_name", input_topic_name_, std::string("/id/pandar/front/compress"));
        this->get_parameter_or("output_topic_name", output_topic_name_, std::string(input_topic_name_ + "/uncompress"));

        auto best_effort_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();

        RCLCPP_INFO(this->get_logger(), "input:  '%s' ", input_topic_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "output: '%s' ", output_topic_name_.c_str());

        subscription_ = this->create_subscription<compressed_pointcloud_interfaces::msg::CompressedPointCloud>(
        input_topic_name_, best_effort_qos_profile, std::bind(&DeCompress::topic_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_name_, 10);
    }

    DeCompress::~DeCompress()
    {}

    void DeCompress::topic_callback(const compressed_pointcloud_interfaces::msg::CompressedPointCloud::SharedPtr msg) const
    {
        sensor_msgs::msg::PointCloud2::Ptr ros_cloud(new sensor_msgs::msg::PointCloud2);
        pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

        std::stringstream compressedData(msg->data);

        // Pack into a compressed message
        try
        {
            /* code */
            cpt.PointCloudDecoder->decodePointCloud(compressedData, pclCloud);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }

        pcl::toROSMsg(*pclCloud, *ros_cloud);
        ros_cloud->header = msg->header;

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
