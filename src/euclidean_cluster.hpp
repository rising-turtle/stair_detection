
#pragma once 


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <vector>
#include <iostream>

template<typename PointT>
bool euclideanClusters(typename pcl::PointCloud<PointT>::Ptr& in, std::vector<typename pcl::PointCloud<PointT>::Ptr>& out, double g_euclidean_threshold=0.02, int g_min_cluster_num=10000)
{
    bool ret = false;
    // Creating kdTree object for the search method of extraction
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); 
    std::vector<pcl::PointIndices> cluster_indices;
    
    std::cout<<"start euclidean clusters: "<<std::endl;

    pcl::EuclideanClusterExtraction<PointT> ec; 
    // ec.setClusterTolerance(8*g_voxel_size);
    ec.setClusterTolerance(g_euclidean_threshold);
    ec.setMinClusterSize(g_min_cluster_num);
    // ec.setMaxClusterSize(g_max_cluster_num);
    ec.setSearchMethod(tree);
    ec.setInputCloud(in);
    ec.extract(cluster_indices);

    std::cout<<"finish euclidean clusters: "<<cluster_indices.size()<<std::endl; 


    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
            it!=cluster_indices.end(); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
        // float max_ = -std::numeric_limits<float>::max(); 
        for(std::vector<int>::const_iterator pit = it->indices.begin(); 
                pit != it->indices.end(); ++pit)
        {
          cluster->points.push_back(in->points[*pit]);
        }
        // cout<<"cluster_extracter.hpp: max_x is : "<<max_<<endl;
        cluster->width = cluster->points.size();
        cluster->height = 1;
        cluster->is_dense = true;
        out.push_back(cluster);
        ret = true;
    }
    return ret;
}
