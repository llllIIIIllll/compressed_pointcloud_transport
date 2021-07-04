
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <pcl_ros/point_cloud.h>

#include "compressed_pointcloud_transport.hpp"
#include <compressed_pointcloud_interfaces/CompressedPointCloud.h>

ros::Publisher pub;
pcl::PointCloud<PointT>::Ptr pclCloud(new pcl::PointCloud<PointT>);

CompressedPointcloudTransport cpt;

void topic_callback (const sensor_msgs::PointCloud2::ConstPtr& rosCloud)
{
    if(pub.getNumSubscribers())
    {
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
        pub.publish(output);
    }
    else
        ROS_DEBUG_NAMED("compressor" ,"Received input cloud but there are no subscribers; not publishing.");
}


int main(int argc, char * argv[])
{

    ros::init(argc, argv, "pointcloud_compress");
    ros::NodeHandle nh;

    std::string input_topic_name  = "/id/pandar/front";
    std::string output_topic_name = "/id/pandar/front/compress";

    nh.getParam("input_topic_name", input_topic_name);
    nh.getParam("output_topic_name", output_topic_name);

    ros::Subscriber sub = nh.subscribe(input_topic_name, 1, topic_callback);
    pub = nh.advertise<compressed_pointcloud_interfaces::CompressedPointCloud>(output_topic_name, 10);

    ros::spin();
    return 0;
}