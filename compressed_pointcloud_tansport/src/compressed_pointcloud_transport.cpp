#include "compressed_pointcloud_transport.hpp"

CompressedPointcloudTransport::CompressedPointcloudTransport()
{
	showStatistics = true;
	compressionProfile = pcl::io::LOW_RES_ONLINE_COMPRESSION_WITHOUT_COLOR;

	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT>(compressionProfile, showStatistics);
	PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT>();
}

CompressedPointcloudTransport::CompressedPointcloudTransport(bool s, pcl::io::compression_Profiles_e e)
: showStatistics(s),
	compressionProfile(e)
{
	PointCloudEncoder = new pcl::io::OctreePointCloudCompression<PointT>(compressionProfile, showStatistics);
	PointCloudDecoder = new pcl::io::OctreePointCloudCompression<PointT>();
}

CompressedPointcloudTransport::~CompressedPointcloudTransport()
{}