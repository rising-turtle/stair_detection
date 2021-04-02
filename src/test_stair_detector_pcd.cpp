/*
	Apr.1 2021, He Zhang, fuyinzh@gmail.com 

	test stair_detector_pcd 

*/

#include "stair_detector_pcd.h"

#include <iostream>
#include <string>

using namespace std; 

string output_pcd_file("out_pc.pcd"); 
string input_pcd_file(""); 

int main(int argc, char* argv[])
{
	if(argc < 2){
		cerr <<"usage: ./test_stair_detector_pcd input.pcd [output.pcd]"<<endl; 
		return -1; 
	}

	input_pcd_file = string(argv[1]); 

	if(argc >= 3)
		output_pcd_file = string(argv[2]); 


	// 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
	int succeed = pcl::io::loadPCDFile(input_pcd_file.c_str(), *in_pc);
	if(succeed != 0){
		cerr <<"test_stair_detector_pcd: failed to load pcd file: "<<input_pcd_file<<endl;
		return -1; 
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZRGB>);

	StairDetectorPCD sd_pcd; 
	double anchor_pt[3]; 
	sd_pcd.detect_stair_pcd(in_pc, out_pc, anchor_pt); 

	pcl::io::savePCDFile(output_pcd_file.c_str(), *out_pc); 


	return 0; 
}