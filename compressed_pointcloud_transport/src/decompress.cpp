#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

//PCL specific includes
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>


#include <compressed_pointcloud_interfaces/CompressedPointCloud.h>

#include "compressed_pointcloud_transport.hpp"

typedef pcl::PointXYZ PointT;

class Decompressor{

protected:
    ros::NodeHandle nh_, pnh_;
    ros::Publisher pub_;
    boost::shared_ptr<ros::Subscriber> sub_;

    std::string input_topic_name_  = "/id/pandar/front";
    std::string output_topic_name_ = "/id/pandar/front/uncompress";

    CompressedPointcloudTransport cpt;

    ros::Timer subscriber_timer_;

public:
    Decompressor()
        :
        nh_("~")
        , pnh_("~")
        // ,pcl_cloud(new pcl::PointCloud<PointT>)
    {

        nh_.getParam("input_topic_name", input_topic_name_);
        nh_.getParam("output_topic_name", output_topic_name_);

        // Create a ROS publisher for the output point cloud
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_name_, 1);

        // Create a ROS subscriber for the input point cloud
        sub_.reset(new ros::Subscriber());
        *sub_ = nh_.subscribe(input_topic_name_, 2, &Decompressor::cloud_cb, this);

        // subscriber_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&Decompressor::timerCB, this));
    }

protected:
    //void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    void cloud_cb (const compressed_pointcloud_interfaces::CompressedPointCloud::ConstPtr& input) 
    {


        // clouds
        pcl::PointCloud<PointT>::Ptr pcl_cloud(new pcl::PointCloud<PointT>);
        sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);

        // Stringstream to retrieve compressed point cloud
        std::stringstream compressed_data(input->data);

        try
        {
            //Decompress the point cloud
            cpt.PointCloudDecoder->decodePointCloud(compressed_data, pcl_cloud);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
        }
        

        //Convert back to sensor_msgs::PointCloud2
        pcl::toROSMsg(*pcl_cloud, *ros_cloud);

        // restore the cloud header
        ros_cloud->header = input->header;

        pub_.publish(ros_cloud);
    }

    // void timerCB()
    // {
    //     if(pub_.getNumSubscribers())
    //     {
    //         if( !sub_ )
    //         {
    //             ROS_INFO("Making a new subscriber!");
    //             sub_.reset(new ros::Subscriber());
    //             *sub_ = nh_.subscribe(input_topic_name_, 2, &Decompressor::cloud_cb, this);
    //         }
    //     }
    //     else
    //     {
    //         if( sub_)
    //         {
    //             ROS_INFO("Deleting subscriber!");
    //             sub_.reset();
    //         }
    //     }
    // }
};

int main(int argc, char** argv) 
{
    // Initialize ROS
    ros::init(argc, argv, "cloud_decompressor");
    Decompressor decomp;

    ros::Duration(1.0).sleep();
    ros::spin();
}
