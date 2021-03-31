
/*
  Mar. 31 2021, He Zhang, fuyinzh@gmail.com 

  generate point cloud from images 

*/


#pragma once 
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <opencv/cv.h>


extern double CX, CY, FX, FY; // camera intrinsics 

extern void generateFilteredPointCloud(const cv::Mat& dpt_img, pcl::PointCloud<pcl::PointXYZ>& pc_out); 
extern void generateColorPointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGB>& pc_out, int skip=1 ) ;
