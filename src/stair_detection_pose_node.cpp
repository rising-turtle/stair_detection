/*
    Mar. 30, 2021, He Zhang, fuyinzh@gmail.com 
    
    receive depth image to detect the staircase and generate point cloud 
    receive current pose to transform the point cloud to world coordinate 

*/


#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <thread>
#include <mutex> 
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>
#include "stair_detector_geo.h"
#include "pc_from_image.h"
#include "transform_pc.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std; 

StairDetectorGeo sdg;
StairDetectorGeoParams param;

std::mutex mutex_img_buf; 
std::condition_variable con; 

queue<sensor_msgs::ImageConstPtr> img_buf;
queue<sensor_msgs::Image::ConstPtr> dpt_img_buf;

string IMAGE_TOPIC = "/keyframe_color_image"; // "/cam0/color";
string DEPTH_TOPIC = "/keyframe_depth_image"; // "/cam0/depth"; 

cv::Mat handle_color_msg(const sensor_msgs::ImageConstPtr &color_msg); 
cv::Mat handle_depth_msg(const sensor_msgs::ImageConstPtr &depth_msg);


ros::Publisher pub_world_pc; 
ros::Publisher pub_detected_stair;


void img_callback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    mutex_img_buf.lock();
    img_buf.push(color_msg); 
    dpt_img_buf.push(depth_msg); 
    mutex_img_buf.unlock(); 
    con.notify_one(); 
}

bool detect_stair(cv::Mat depth_image, std::vector<cv::Point>& bounding_box ){
    static int times = 0; 
    static int times_threshold = 3; // consecutive times larger than this threshold 

    if(sdg.getStairs(depth_image, bounding_box)){
        ROS_INFO("stair_detection_pose_node: found potential stair, count %d", ++times); 
        if(times >= times_threshold){
            ROS_DEBUG("stair_detection_pose_node: succeed to detect stair"); 

            // further check out 



            return true; 
        }
        return false; 
    }else{
        times = 0; 
    }
    return false; 
}

void pcd_thread()
{
    tf::TransformListener listener; 

    bool found_stair = false; 
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_pc_stair(new pcl::PointCloud<pcl::PointXYZRGB>);    

    while(ros::ok()){

        std::unique_lock<std::mutex> lk(mutex_img_buf); 
        queue<sensor_msgs::ImageConstPtr> img_buf_proc;
        queue<sensor_msgs::Image::ConstPtr> dpt_img_buf_proc;
        con.wait(lk, [&]
            {
                img_buf_proc.swap(img_buf); 
                dpt_img_buf_proc.swap(dpt_img_buf); 
                return (img_buf_proc.size() > 0); 
             });
        lk.unlock(); 

        ros::Time msg_time = img_buf_proc.back()->header.stamp;
        // ROS_ERROR("stair_detection_with_pose: receive rgb and depth at %lf", msg_time.toSec());

        cv::Mat img_m = handle_color_msg(img_buf_proc.back()); 
        cv::Mat dpt_m = handle_depth_msg(dpt_img_buf_proc.back()); 

        if(!img_m.data || !dpt_m.data){
            ROS_ERROR("stair_detection_pose_node: receive img's data is NULL!");
            break; 
        }

        // wait for the corresponding TF 
        tf::StampedTransform transform;
        try{
            listener.waitForTransform("world", "camera",
                              msg_time, ros::Duration(2.0));
            listener.lookupTransform("world", "camera",  
                               msg_time, transform);
            ROS_DEBUG("stair_detection_with_pose: receive Tw2c translation: %lf %lf %lf at %lf", 
                transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z(), transform.stamp_.toSec()); 
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            // ros::Duration(1.0).sleep();
            continue; 
        }

        // cv::imshow("Color_image", img_m);
        // cv::waitKey(5);

        // cv::imshow("Depth_image", dpt_m);
        // cv::waitKey(5); 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_world(new pcl::PointCloud<pcl::PointXYZRGB>); 
        if(!found_stair){
            std::vector<cv::Point> bounding_box;
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc_cam(new pcl::PointCloud<pcl::PointXYZRGB>); 

            if(detect_stair(dpt_m, bounding_box)){
                // generate pcd 
                generateColorPointCloudBoundingBox(img_m, dpt_m, *pc_cam,bounding_box, 2);

                // publish the results 
                sdg.drawBox(img_m, bounding_box);
                sensor_msgs::Image img_msg; // >> message to be sent
                std_msgs::Header header; // empty header
                header.stamp = msg_time;  
                cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_m);
                img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
                pub_detected_stair.publish(img_msg); 

                // change the color of the detected stair 
                markColor(*pc_cam, PURPLE);             
                
                // transform into world coordinate 
                detected_pc_stair = transform_pc(pc_cam, transform);
                pc_world = detected_pc_stair; 
                found_stair = true; 
                pcl::io::savePCDFile("potential_stair_box.pcd", *detected_pc_stair); 

            }  else{
                // generate pcd 
                generateColorPointCloud(img_m, dpt_m, *pc_cam, 2) ;

                // transform into world coordinate 
                pc_world = transform_pc(pc_cam, transform);
            }
        }else{
                pc_world = detected_pc_stair; 
        }

        // publish it  
        sensor_msgs::PointCloud2 pc_world_msg;
        pcl::toROSMsg(*(pc_world), pc_world_msg); 
        pc_world_msg.header.frame_id = "world";  
        pc_world_msg.header.stamp = msg_time;
        pub_world_pc.publish(pc_world_msg);
    
        ros::spinOnce(); 
    }

}

void tf_listener(){


    tf::TransformListener listener; 

    double timestamp = 0; 

    while(ros::ok()){

    tf::StampedTransform transform;
   
        try{
            listener.lookupTransform("world", "camera",  
                               ros::Time(0), transform);
            if(transform.stamp_.toSec() > timestamp){
                ROS_ERROR("receive Tw2c translation: %lf %lf %lf at %lf ", transform.getOrigin().x(), transform.getOrigin().y(), 
                    transform.getOrigin().z(), transform.stamp_.toSec()); 
                timestamp = transform.stamp_.toSec(); 
            }
            // ros::Duration(3*1e-3).sleep(); 
        }
        catch (tf::TransformException ex){
            // ROS_ERROR("%s",ex.what());
            // ros::Duration(1.0).sleep();
        }
        ros::Duration(3*1e-3).sleep(); 
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "stair_detection_with_pose_node");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    // cv::namedWindow("Depth_image");
    // cv::namedWindow("Color_image"); 
    // cv::startWindowThread();

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
    //     message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync(sub_image, sub_depth, 100);
    //     use ApproximateTime to fit fisheye camera
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&img_callback, _1, _2));


    // setup publishers 
    pub_world_pc = n.advertise<sensor_msgs::PointCloud2>("/current_point_cloud", 1000); 

    pub_detected_stair = n.advertise<sensor_msgs::Image>("/stair_detection_results", 1000); 

    std::thread pcd_thread_{pcd_thread}; 
    // std::thread tf_listener_{tf_listener}; 

    ros::spin();
    // cv::destroyWindow("Depth_image");
    // cv::destroyWindow("Color_image");
    return 0;
}


cv::Mat handle_color_msg(const sensor_msgs::ImageConstPtr &color_msg)
{
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
    return ret_img; 
}

cv::Mat handle_depth_msg(const sensor_msgs::ImageConstPtr &depth_msg)
{
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

    return depth_ptr->image; 
}


