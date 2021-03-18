/*
	Test with input images 
*/

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "stair_detector_geo.h"

using namespace std; 

StairDetectorGeo sdg;
StairDetectorGeoParams param;

string img_file = ""; 

int main(int argc, char **argv)
{

	if(argc >= 2){
		img_file = argv[1]; 
	}

    // param.debug = false;
    param.fill_invalid = true;
    param.ignore_invalid = false;
    param.use_laplacian = false;
    param.minimum_line_num = 10;
    param.neighbour_count = 3; 
    param.debug = true; 
    sdg.setParam(param);

	cout <<" img_file: "<<img_file<<endl; 

	cv::Mat img = cv::imread(img_file.c_str(), -1); 
	cv::Mat gray_img; 
	cv::Mat show_img; 

	if(img.channels() == 3){
		cout<<" input a 8UC3 mat, convert to gray value" <<endl; 
		cv::cvtColor(img, gray_img, CV_RGB2GRAY);
		show_img = img; 

		// change Canny's threshold 
		param.canny_low_threshold = 7; 
		// param.canny_ratio = 10;

	}else{
		gray_img = img.clone(); 
		cv::cvtColor(img, show_img, CV_GRAY2RGB); 
	}

	std::vector<cv::Point> show_bounding_box;

	sdg.setParam(param);

  	if (sdg.getStairs(gray_img, show_bounding_box)) {
          
        std::cout << "Found Potential Stiars" << std::endl;
        sdg.drawBox(show_img, show_bounding_box);
        
        cv::imshow("Result", show_img);
        cv::waitKey(0);

    } else {
         
        std::cout << "Can't find stiars" << std::endl;
    }
  
    return 0;
}

