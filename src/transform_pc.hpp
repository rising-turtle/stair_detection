/*
	Mar. 31 2021, He Zhang, fuyinzh@gmail.com 

	transform point clouds 

*/


#pragma once 

#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/io.h>
#include <tf/tf.h>

template <typename T>
boost::shared_ptr<pcl::PointCloud<T>> transform_pc(const boost::shared_ptr<pcl::PointCloud<T> >& pc_in, tf::Transform Tji)
{
	typename boost::shared_ptr<pcl::PointCloud<T>> ret(new pcl::PointCloud<T>); 

	copyPointCloud(*pc_in, *ret); 

	for(int i=0; i<pc_in->points.size(); i++){

		const T& pt_in = pc_in->points[i]; 
		tf::Vector3 pii(pt_in.x, pt_in.y, pt_in.z); 
		tf::Vector3 pjj = Tji * pii; 
		T& pt_out = ret->points[i]; 
		pt_out.x = pjj.getX(); 
		pt_out.y = pjj.getY(); 
		pt_out.z = pjj.getZ(); 
	}
	return ret; 
}