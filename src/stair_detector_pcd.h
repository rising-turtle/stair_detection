/*
	
	Apr. 1 2021, He Zhang, fuyinzh@gmail.com 
	
	wrapper of the point cloud based stair detector 

*/

#pragma once 

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

class StairDetectorPCD
{
public:
	StairDetectorPCD(); 
	~StairDetectorPCD(); 

	bool detect_stair_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> >& in_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc, double* central_pt = 0); 
	bool detect_stair_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& in_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc, double* central_pt = 0); 

};