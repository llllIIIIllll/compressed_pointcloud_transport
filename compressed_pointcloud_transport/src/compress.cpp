
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/compression/octree_pointcloud_compression.h>

#include <pcl_ros/point_cloud.h>

#include "compressed_pointcloud_transport.hpp"
#include <compressed_pointcloud_interfaces/CompressedPointCloud.h>

class Compressor{

protected:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_;
    boost::shared_ptr<ros::Subscriber> sub_;

    std::string input_topic_name_  = "/id/pandar/front";
    std::string output_topic_name_ = "/id/pandar/front/compress";

    CompressedPointcloudTransport cpt;

    ros::Timer subscriber_timer_;

public:
    Compressor()
        :
        nh_("~")
        , pnh_("~")
        // ,pcl_cloud(new pcl::PointCloud<PointT>)
    {

        nh_.getParam("input_topic_name", input_topic_name_);
        nh_.getParam("output_topic_name", output_topic_name_);

        // Create a ROS publisher for the output point cloud
        pub_ = nh_.advertise<compressed_pointcloud_interfaces::CompressedPointCloud>(output_topic_name_, 1);

        // Create a ROS subscriber for the input point cloud
        sub_.reset(new ros::Subscriber());
        *sub_ = nh_.subscribe(input_topic_name_, 2, &Compressor::topic_callback, this);
    }

protected:
    //void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    void topic_callback (const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
    {
        pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);
        // Stringstream to store compressed point cloud
        std::stringstream compressedData;

        // Must convert from sensor_msg::PointCloud2 to pcl::PointCloud<PointT> for the encoder
        pcl::fromROSMsg(*rosCloud, *pclCloud);

        // Compress the pointcloud
        cpt.PointCloudEncoder->encodePointCloud (pclCloud, compressedData);

        // Pack into a compressed message
        compressed_pointcloud_interfaces::CompressedPointCloud output;
        output.header = rosCloud->header;
        output.data = compressedData.str();
        pub_.publish(output);
    }
};


int main(int argc, char * argv[])
{
    // Initialize ROS
    ros::init(argc, argv, "cloud_compressor");
    Compressor comp;

    ros::Duration(1.0).sleep();
    ros::spin();
    return 0;
}