
/*
	Apr. 9 2021, He Zhang, fuyinzh@gmail.com 

	downsample point clouds 

*/


#pragma once 

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <tf/tf.h>
#include <pcl/filters/voxel_grid.h>

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> downsample_pc(const boost::shared_ptr<pcl::PointCloud<T> >& pc_in, float _voxel_size=0.05)
{
	typename boost::shared_ptr<pcl::PointCloud<T>> ret(new pcl::PointCloud<T>); 

  	// voxelgrid filter
    typename pcl::VoxelGrid<T> vog;
    vog.setInputCloud(pc_in); 
    vog.setLeafSize(_voxel_size, _voxel_size, _voxel_size);
    vog.filter(*ret);

	return ret; 
}