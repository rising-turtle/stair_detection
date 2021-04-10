/*
	
	Apr. 1 2021, He Zhang, fuyinzh@gmail.com 
	
	wrapper of the point cloud based stair detector 

*/


#include "stair_detector_pcd.h"

#include <stairs/preanalysis.h>
#include <stairs/regions.h>
#include <stairs/regiongrowing.h>
#include <stairs/voxSAC.h>
#include <stairs/splitmerge.h>
#include <stairs/planeshape.h>
#include <stairs/recognition.h>
#include <stairs/StairVector.h>
#include <iostream>
#include <vector>
#include <map>
#include "euclidean_cluster.hpp"

using namespace std; 

typedef pcl::PointXYZ PointT;
typedef pcl::PointNormal PointNT;
typedef pcl::Normal Normal;
typedef pcl::PointXYZRGB PointTC;

typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::PointCloud<PointNT> PointCloudN;
typedef pcl::PointCloud<Normal> NormalCloud;
typedef pcl::PointCloud<PointTC> PointCloudC;

StairDetectorPCD::StairDetectorPCD(){}
StairDetectorPCD::~StairDetectorPCD(){}

bool StairDetectorPCD::detect_stair_pcd(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB> >& in_pc, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_pc, double* central_pt){

	PointCloudT::Ptr new_pc(new PointCloudT); 
	new_pc->points.resize(in_pc->points.size()); 
	for(int i=0; i< new_pc->points.size(); i++){
		PointT& new_pt = new_pc->points[i]; 
		PointTC& in_pt = in_pc->points[i]; 
		new_pt.x = in_pt.x; 
		new_pt.y = in_pt.y; 
		new_pt.z = in_pt.z; 
	}
	new_pc->height = in_pc->height; 
	new_pc->width = in_pc->width; 
	new_pc->is_dense = in_pc->is_dense; 

	return detect_stair_pcd(new_pc, out_pc, central_pt); 

}

bool StairDetectorPCD::detect_stair_pcd(PointCloudT::Ptr& in_pc, PointCloudC::Ptr& out_pc, double* central_pt)
{

	// preanalysis
	std::cout<<"stair_detector_pcd: Starting preanalysis"<<std::endl;
    double preAS = pcl::getTime();

	Preanalysis pre;
	pre.dsFlag = false; // no downsample 
	pre.neMethod = 0; // no point filter 
    NormalCloud::Ptr prepNomalCloud;
    prepNomalCloud.reset(new NormalCloud);
    PointCloudT floorPC;
    PointCloudC prepNorMap;
    pre.run(in_pc, prepNomalCloud, prepNorMap, floorPC);
    double preAE = pcl::getTime();
    std::cout<<"stair_detector_pcd: Preanalysis took: "<<preAE-preAS<<std::endl;


    // segmentation 
    double segS = pcl::getTime();
    regions segRegions;
    RegionGrowing reGrow;
    reGrow.setInputCloud(in_pc);
    reGrow.setNormalCloud(prepNomalCloud);
    reGrow.run(segRegions);
    double segE = pcl::getTime();
    std::cout<<"stair_detector_pcd: Segmentation took: "<<segE-segS<<std::endl;

    // plane extraction 
    double pfS = pcl::getTime();
    planeshape psProc;
    regions stairTreads;
    regions stairRisers;
    psProc.setInputRegions(segRegions);
    psProc.filterSc(stairTreads, stairRisers);

    double pfE = pcl::getTime();
    std::cout<<"stair_detector_pcd: Plane filter took: "<<pfE-pfS<<std::endl;

    std::cout<<"stair_detector_pcd: There are treads: "<<stairTreads.size()<<std::endl;
    std::cout<<"stair_detector_pcd: There are risers: "<<stairRisers.size()<<std::endl;

    if(stairTreads.size() < 3){
    	std::cout<<"stair_detector_pcd: There are too few treads: "<<stairTreads.size()<<std::endl;
    	return false; 
    }

    // for(int i=0; i<stairTreads.size(); i++){
    // 	PointCloudC regionPC = stairTreads.getColoredCloud(); 
    // 	*out_pc += regionPC; 
    // }
    // for(int i=0; i<stairRisers.size(); i++){
    // 	PointCloudC regionPC = stairRisers.getColoredCloud(); 
    // 	*out_pc += regionPC; 
    // }


    // clusters go, but too slow 
    // std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> out_clusters; 
    // if(euclideanClusters<pcl::PointXYZRGB>(out_pc, out_clusters)){
    // 	std::cout<<" stair_detector_pcd: extract "<<out_clusters.size()<<" clusters. "<<std::endl; 
    // 	for(int k=0; k<out_clusters.size(); k++){
    // 		std::cout<<"cluster "<<k<<" has "<<out_clusters[k]->size()<<" points!"<<std::endl; 
    // 	}
    // 	out_pc.swap(out_clusters[0]); 
    // }else{
    // 	std::cout<<"stair_detector_pcd: failed to find any clusters"<<std::endl; 
    // }


    // graph refinement 
    StairVector detectedStairs;
    double refS = pcl::getTime();
    recognition stairDetect;
    stairDetect.setInputRegions(segRegions);
    stairDetect.setStairTreadRegions(stairTreads);
    stairDetect.setStairRiseRegions(stairRisers);
    stairDetect.run(detectedStairs);
    double refE = pcl::getTime();
    std::cout<<"Refinement took: "<<refE-refS<<std::endl;

    std::cout<<"Detected stairways: "<<detectedStairs.size()<<std::endl;
    bool colorByPart = true;
    if(detectedStairs.size()>0 && detectedStairs.at(0).stairParts.size() > 3) // 0
    {
		for(int stairCoeffIdx =0; stairCoeffIdx < detectedStairs.size(); stairCoeffIdx++)
		{
			Stairs stairCoefficients;
			stairCoefficients = detectedStairs.at(stairCoeffIdx);

			// float steigung = atan(stairCoefficients.dir[2] / sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)));

			// std::cout<<std::endl<<"Step depth:   "<<round(1000*sqrt(pow(stairCoefficients.dir[0],2) + pow(stairCoefficients.dir[1],2)))<<std::endl;
			// std::cout<<"Step height:  "<<round(1000*stairCoefficients.dir[2])<<std::endl;
			// std::cout<<"Step width:   "<<round(1000*stairCoefficients.width)<<std::endl;
			// std::cout<<"Slope is:     "<<round(100*steigung/M_PI*180)<<std::endl;
			// std::cout<<"Amount of stair parts: "<<stairCoefficients.size()<<std::endl<<std::endl;

			// float stairAngle = atan2(stairCoefficients.dir[1],stairCoefficients.dir[0]);
			// float xStairDist = stairCoefficients.pos[0];
			// float yStairDist = stairCoefficients.pos[1];

			// Eigen::Vector2f sepDist;
			// sepDist[0] = cos(stairAngle) * xStairDist + sin(stairAngle) * yStairDist;
			// sepDist[1] = - sin(stairAngle) * xStairDist + cos(stairAngle) * yStairDist;

	  //       std::cout<<"Dist in X is: "<<round(1000*(stairCoefficients.pos[0]))<<std::endl;
	  //       std::cout<<"Dist in Y is: "<<round(1000*stairCoefficients.pos[1])<<std::endl;

	  //       std::cout<<"Dist par is:  "<<round(1000*sepDist[0])<<std::endl;
	  //       std::cout<<"Dist ort is:  "<<round(1000*sepDist[1])<<std::endl;
			// std::cout<<"Anchor point is: "<<stairCoefficients.anchPoint<<std::endl;

			// std::cout<<"Angle is:     "<<round(100*atan2(stairCoefficients.dir[1],stairCoefficients.dir[0])/M_PI*180)<<std::endl;

			if(colorByPart)
				*out_pc += detectedStairs.getColoredParts(stairCoeffIdx);
			else
				*out_pc += detectedStairs.getColoredCloud(stairCoeffIdx);

		    if(central_pt != NULL){
		    	// stairCoefficients.stairTreads.generateCenterCloud(); 
		    	stairCoefficients.stairParts.generateCenterCloud();
		    	map<double, int> vz; 
		    	for(int i=0; i<stairCoefficients.stairParts.centerCloud.size(); i++){
		    		PointT& pt = stairCoefficients.stairParts.centerCloud.points[i]; 
		    	 	std::cout<<"Treads "<<i<<" center point: "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl; 
		    	 	vz[pt.z] = i; 
		    	}

		    	map<double, int>::iterator it = vz.begin(); 
		    	// it++; it++; 

		    	// PointT& pt = stairCoefficients.stairTreads.centerCloud.points[stairCoefficients.anchPoint]; 
		    	PointT& pt = stairCoefficients.stairParts.centerCloud.points[it->second]; 
		    	central_pt[0] = pt.x; central_pt[1] = pt.y; central_pt[2] = pt.z; 
		    	cout <<"The smallest is: "<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl; 

		    	// PointT& pt = stairCoefficients.stairTreads.centerCloud.points[stairCoefficients.anchPoint]; 
		    	// central_pt[0] = pt.x; central_pt[1] = pt.y; central_pt[2] = pt.z; 
		    	//  find the third smallest z axis 
		    	


		    }
		}
    }else{
    	return false;
    }

    out_pc->height = 1;
    out_pc->width = out_pc->points.size(); 
    out_pc->is_dense = true; 

    // if(central_pt != NULL){
    // 	double sx = 0; double sy = 0; double sz = 0; 

    // 	for(int i=0; i<out_pc->size(); i++){
    // 		sx += out_pc->points[i].x; 
    // 		sy += out_pc->points[i].y; 
    // 		sz += out_pc->points[i].z; 
    // 	}
    // 	central_pt[0] = sx / out_pc->size(); 
    // 	central_pt[1] = sy / out_pc->size(); 
    // 	central_pt[2] = sz / out_pc->size(); 
    // 	std::cout<<"central_pt: "<<central_pt[0]<<" "<<central_pt[1]<<" "<<central_pt[2]<<endl;
    // }

    return true; 

}