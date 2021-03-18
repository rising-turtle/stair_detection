/*
    Mar. 16, 2021, He Zhang, fuyinzh@gmail.com 
    
    receive camera data to detect the staircase 

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

queue<sensor_msgs::ImageConstPtr> img_buf;
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

string IMAGE_TOPIC = "/cam0/color";
string DEPTH_TOPIC = "/cam0/depth"; 

void img_callback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = color_msg->header.stamp.toSec();
        last_image_time = color_msg->header.stamp.toSec();
        return;
    }
    // detect unstable camera stream
    if (color_msg->header.stamp.toSec() - last_image_time > 1.0 || color_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true;
        last_image_time = 0;
        pub_count = 1;
        return;
    }
    last_image_time = color_msg->header.stamp.toSec();

    // encodings in ros: http://docs.ros.org/diamondback/api/sensor_msgs/html/image__encodings_8cpp_source.html
    //color has encoding RGB8
    cv_bridge::CvImageConstPtr ptr;
    cv::Mat ret_img; 
    if (color_msg->encoding == "8UC1")//shan:why 8UC1 need this operation? Find answer:https://github.com/ros-perception/vision_opencv/issues/175
    {
        sensor_msgs::Image img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        ret_img = ptr->image; 
    } else if(color_msg->encoding == "8UC3"){
        sensor_msgs::Image img;
        img.header = color_msg->header;
        img.height = color_msg->height;
        img.width = color_msg->width;
        img.is_bigendian = color_msg->is_bigendian;
        img.step = color_msg->step;
        img.data = color_msg->data;
        img.encoding = "bgr8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        ret_img = ptr->image.clone(); 
        // cv::cvtColor(ret_img, ret_img, cv::COLOR_BGR2GRAY);
    }
    else{
        ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
        ret_img = ptr->image.clone(); 
    }

    //depth has encoding TYPE_16UC1
    cv_bridge::CvImageConstPtr depth_ptr;
    // debug use     std::cout<<depth_msg->encoding<<std::endl;
    if (depth_msg->encoding == "16UC1")
    {
        sensor_msgs::Image img;
        img.header = depth_msg->header;
        img.height = depth_msg->height;
        img.width = depth_msg->width;
        img.is_bigendian = depth_msg->is_bigendian;
        img.step = depth_msg->step;
        img.data = depth_msg->data;
        img.encoding = "mono16"; // sensor_msgs::image_encodings::MONO16;
        depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
    }else
        depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO16);


    cv::Mat show_img = ret_img; //ptr->image;
    cv::Mat dep_img = depth_ptr->image.clone(); 


   cv::imshow("Color_image", show_img);
   cv::waitKey(5);
   // cv::imshow("dep", dep_img); 
   // cv::waitKey(5); 


    std::vector<cv::Point> show_bounding_box;
    try
    {
        // cv::Mat depth_image = cv_bridge::toCvCopy(msg, "8UC1")->image;
        cv::Mat depth_image = dep_img;
        // dep_img.convertTo(depth_image, CV_8UC1, 255./5000.); // maximum 5 meters 

        cv::imshow("Depth_image", depth_image); 
        cv::waitKey(10);
        if (sdg.getStairs(depth_image, show_bounding_box)) {
          
            std::cout << "Found Potential Stiars" << std::endl;
            sdg.drawBox(show_img, show_bounding_box);
        
            cv::imshow("Result", show_img);
            cv::waitKey(10);

        } else {
         
            std::cout << "Can't find stiars" << std::endl;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        // ROS_ERROR("Could not convert from '%s' to '8UC1'.", msg->encoding.c_str());
    }


    // init pts here, using readImage()
    ROS_INFO("depth_image size: width: %d height: %d", dep_img.cols, dep_img.rows);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_detection_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    cv::namedWindow("Result");
    cv::namedWindow("Depth_image");
    cv::namedWindow("Color_image"); 
    cv::startWindowThread();

    // param.debug = false;
    param.fill_invalid = true;
    param.ignore_invalid = false;
    param.use_laplacian = false;
    param.minimum_line_num = 10;
    param.neighbour_count = 3; 
    param.debug = false; 
    sdg.setParam(param);

    //ref: http://docs.ros.org/api/message_filters/html/c++/classmessage__filters_1_1TimeSynchronizer.html#a9e58750270e40a2314dd91632a9570a6
    //     https://blog.csdn.net/zyh821351004/article/details/47758433
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
//    message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_depth, 100);
    // use ApproximateTime to fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));

    ros::spin();
    cv::destroyWindow("Result");
    cv::destroyWindow("Depth_image");
    cv::destroyWindow("Color_image");
    return 0;
}


// new points velocity is 0, pub or not?
// track cnt > 1 pub?
