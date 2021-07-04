#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <iostream>
#include <vector>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>

//PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/compression/octree_pointcloud_compression.h>


typedef pcl::PointXYZ PointT;

class CompressedPointcloudTransport
{

public:
	CompressedPointcloudTransport();
	CompressedPointcloudTransport(bool s, pcl::io::compression_Profiles_e e);
	~CompressedPointcloudTransport();

    pcl::io::OctreePointCloudCompression<PointT>* PointCloudEncoder;
    pcl::io::OctreePointCloudCompression<PointT>* PointCloudDecoder;

private:
	bool showStatistics;
	pcl::io::compression_Profiles_e compressionProfile;

};