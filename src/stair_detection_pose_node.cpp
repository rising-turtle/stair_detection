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
#include "stair_detector_pcd.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "downsample_pc.hpp"

using namespace std; 

#define R2D(r) ((r)*180./3.141592654)
#define SQ(x) (((x)*(x)))

StairDetectorGeo sdg;
StairDetectorGeoParams param;

StairDetectorPCD sd_pcd; 

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
ros::Publisher pub_stair_dir;
ros::Publisher pub_stair_loc; 

void print_tf(tf::Vector3 t, string pre=""){
    cout<<pre<<"\t"<<t.getX()<<"\t"<<t.getY()<<"\t"<<t.getZ()<<endl; 
}

// parameters 
int times_threshold = 3; 

void img_callback(const sensor_msgs::ImageConstPtr &color_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    mutex_img_buf.lock();
    img_buf.push(color_msg); 
    dpt_img_buf.push(depth_msg); 
    mutex_img_buf.unlock(); 
    con.notify_one(); 
}

bool detect_stair(cv::Mat depth_image, std::vector<cv::Point>& bounding_box );

std::vector<cv::Point> back_project(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_c, tf::Transform Twc); 
std::vector<cv::Point> find_bounding_box(std::vector<cv::Point>& pts);

void pcd_thread()
{
    tf::TransformListener listener; 

    bool found_stair = false; 
    double anchor_pt[3]; 
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr detected_pc_stair(new pcl::PointCloud<pcl::PointXYZRGB>);   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_pc_stair(new pcl::PointCloud<pcl::PointXYZRGB>);    

    double IMAGE_AREA = IMAGE_HEIGHT*IMAGE_WIDTH; 

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
            // ROS_ERROR("%s",ex.what());
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

                // further check out with pcds
                pc_world = transform_pc(pc_cam, transform); 

                pcl::PointCloud<pcl::PointXYZRGB>::Ptr out_pc(new pcl::PointCloud<pcl::PointXYZRGB>); 
                if(sd_pcd.detect_stair_pcd(pc_world, out_pc, &anchor_pt[0])){
                    
                    // // publish the results 
                    // sdg.drawBox(img_m, bounding_box);
                    // sensor_msgs::Image img_msg; // >> message to be sent
                    // std_msgs::Header header; // empty header
                    // header.stamp = msg_time;  
                    // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_m);
                    // img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
                    // pub_detected_stair.publish(img_msg); 

                    // change the color of the detected stair 
                    // markColor(*pc_cam, PURPLE);             
                    
                    // transform into world coordinate 
                    // detected_pc_stair = transform_pc(pc_cam, transform);
                    detected_pc_stair = out_pc; 
                    pc_world = detected_pc_stair; 
                    found_stair = true; 
                   //  markColor(*detected_pc_stair, RED);
                   // pcl::io::savePCDFile("detected_stair_box.pcd", *detected_pc_stair); 
                    downsampled_pc_stair = downsample_pc(pc_world, 0.2); 
                    // markColor(*downsampled_pc_stair, RED);

                    ROS_INFO("stair_detection_pose_node: stair location: %lf %lf %lf", anchor_pt[0], anchor_pt[1], anchor_pt[2]); 
                }

            }  else{
                // generate pcd 
                generateColorPointCloud(img_m, dpt_m, *pc_cam, 2) ;

                // transform into world coordinate 
                pc_world = transform_pc(pc_cam, transform);
            }
        }else{
                pc_world = detected_pc_stair; 

                // we know anchor point and current camera pose  
                tf::Transform Tiw = transform.inverse(); 
                tf::Vector3 stair_pt_w(anchor_pt[0], anchor_pt[1], anchor_pt[2]); 
                tf::Vector3 stair_pt_c = Tiw * stair_pt_w; 
                tf::Vector3 cam_loc_c(0, 0, 0); 
                tf::Vector3 cam_loc_w = transform * cam_loc_c;

                tf::Vector3 cam_loc(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()); 
                tf::Vector3 cam_to_stair_in_w = stair_pt_w - cam_loc; 
                tf::Vector3 cam_to_stair_in_c = Tiw.getBasis() * cam_to_stair_in_w;

                // print_tf(cam_loc_w, "cam_loc_w"); 
                // print_tf(cam_loc, "cam_loc"); 
                // print_tf(stair_pt_w, "stair_pt_w"); 
                // print_tf(stair_pt_c, "stair_pt_c"); 
                // print_tf(cam_to_stair_in_w, "cam_to_stair_in_w"); 
                // print_tf(cam_to_stair_in_c, "cam_to_stair_in_c");

                tf::Vector3 heading_dir(0, 0, 1); 
                tf::Vector3 heading_in_w = transform.getBasis() * heading_dir; 

                // double theta1 = heading_dir.angle(stair_pt_c); 

                heading_in_w.setZ(0);  cam_to_stair_in_w.setZ(0) ; 
                double theta = heading_in_w.angle(cam_to_stair_in_w); 
                double distance = stair_pt_c.length(); 
                double distance_horizontal = cam_to_stair_in_w.length();
                

                // cout <<"theta1: "<<R2D(theta1)<<" theta: "<<R2D(theta)<<endl; 
                // cout <<"stair_pt_c: "<<stair_pt_c.getX()/distance<<"\t"<<stair_pt_c.getY()/distance<<"\t"<<stair_pt_c.getZ()/distance<<endl;
                // cout <<"cam_to_stair_in_c: "<<cam_to_stair_in_c.getX()/cam_to_stair_in_c.length()<<"\t"<<cam_to_stair_in_c.getY()/cam_to_stair_in_c.length()
                    // <<"\t"<<cam_to_stair_in_c.getZ()/cam_to_stair_in_c.length()<<endl;

                static ofstream ouf("motion_feature.log"); 
                cout <<"stair_detection_pose_node.cpp: detect stair, angle: "<< R2D(theta)<<" horizontal distance: "<< sqrt(SQ(stair_pt_w.getX() - transform.getOrigin().getX()) 
                    + SQ(stair_pt_w.getY() - transform.getOrigin().getY())) <<endl;


                // compute bounding box of the stair in currect image plane 
                vector<cv::Point> proj_2d_pts = back_project(downsampled_pc_stair, Tiw); 

                if(proj_2d_pts.size() >=2){

                    std::vector<cv::Point> bx = find_bounding_box(proj_2d_pts);

                    // compute center and normalized bx area
                    cv::Point left_up = bx[0]; 
                    cv::Point right_down = bx[1]; 

                    cv::Point bx_center((left_up.x+right_down.x)/2, (left_up.y+right_down.y)/2); 
                    double bx_area = (right_down.x - left_up.x) * (right_down.y - left_up.y); 

                    ouf<<std::fixed<<msg_time.toSec()<<"\t"<<distance_horizontal<<"\t"<<R2D(theta)<<"\t"<<stair_pt_c.getX()/distance<<
                            "\t"<<stair_pt_c.getY()/distance<<"\t"<<stair_pt_c.getZ()/distance<<"\t"
                            <<bx_center.x<<"\t"<<bx_center.y<<"\t"<<(right_down.x - left_up.x)<<"\t"<<(right_down.y - left_up.y)<<"\t"<<(bx_area/IMAGE_AREA)<<endl; 

                     // publish the results 
                    sdg.drawBox(img_m, bx);
                    sensor_msgs::Image img_msg; // >> message to be sent
                    std_msgs::Header header; // empty header
                    header.stamp = msg_time;  
                    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img_m);
                    img_bridge.toImageMsg(img_msg); // from cv_bridge to sensor_msgs::Image
                    pub_detected_stair.publish(img_msg); 

                }else{

                    ROS_WARN("stair_detection_pose_node.cpp: stair is out of camera view!"); 
                    ouf<<std::fixed<<msg_time.toSec()<<"\t"<<distance<<"\t"<<R2D(theta)<<"\t"<<stair_pt_c.getX()/distance<<
                            "\t"<<stair_pt_c.getY()/distance<<"\t"<<stair_pt_c.getZ()/distance<<"\t"
                            <<-1<<"\t"<<-1<<"\t"<<0<<"\t"<<0<<"\t"<<0<<endl; 

                }

                // publish stair direction  
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header.frame_id = "world"; //  "camera"; // 
                arrow_marker.header.stamp = msg_time; 
                arrow_marker.ns = "basic_shapes";
                arrow_marker.id = 0; 
                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                arrow_marker.type = visualization_msgs::Marker::ARROW; // CUBE
                arrow_marker.action = visualization_msgs::Marker::ADD;
                geometry_msgs::Point p;
                // p.x = p.y = p.z = 0; 
                p.x = transform.getOrigin().getX(); p.y = transform.getOrigin().getY(); p.z = transform.getOrigin().getZ(); 
                arrow_marker.points.push_back(p); 
                // p.x = stair_pt_c.getX(); p.y = stair_pt_c.getY(); p.z = stair_pt_c.getZ(); 
                p.x = stair_pt_w.getX(); p.y = stair_pt_w.getY(); p.z = stair_pt_w.getZ(); 
                arrow_marker.points.push_back(p); 
                arrow_marker.pose.orientation.w = 1.0; 
                arrow_marker.scale.x = 0.05;
                arrow_marker.scale.y = 0.05;
                arrow_marker.scale.z = 0.05;

                arrow_marker.color.b = 1.0f;
                arrow_marker.color.r = 1.0f; 
                arrow_marker.color.a = 1.0f; 

                pub_stair_dir.publish(arrow_marker);

                // publish stair location 
                visualization_msgs::Marker sphere_marker;
                sphere_marker.header.frame_id = "world"; 
                sphere_marker.header.stamp = msg_time; 
                sphere_marker.ns = "basic_shapes";
                sphere_marker.id = 1; 
                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                sphere_marker.type = visualization_msgs::Marker::SPHERE; // CUBE
                sphere_marker.action = visualization_msgs::Marker::ADD;

                sphere_marker.pose.position.x = anchor_pt[0];
                sphere_marker.pose.position.y = anchor_pt[1];
                sphere_marker.pose.position.z = anchor_pt[2];
                sphere_marker.pose.orientation.x = 0.0;
                sphere_marker.pose.orientation.y = 0.0;
                sphere_marker.pose.orientation.z = 0.0;
                sphere_marker.pose.orientation.w = 1.0;
                sphere_marker.scale.x = 0.2; // 0.2 meters
                sphere_marker.scale.y = 0.2; 
                sphere_marker.scale.z = 0.2; 

                sphere_marker.color.g = 1.0f;
                sphere_marker.color.r = 1.0f; 
                sphere_marker.color.a = 1.0f; 

                pub_stair_loc.publish(sphere_marker);
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
    param.text_debug = true; 

    n.param("times_threshold", times_threshold, times_threshold);
    n.param("minimum_line_num", param.minimum_line_num, param.minimum_line_num);  

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

    pub_stair_dir = n.advertise<visualization_msgs::Marker>("visual_stair_direction", 7); 
    pub_stair_loc = n.advertise<visualization_msgs::Marker>("visual_stair_location", 7); 

    std::thread pcd_thread_{pcd_thread}; 
    // std::thread tf_listener_{tf_listener}; 

    ros::spin();
    // cv::destroyWindow("Depth_image");
    // cv::destroyWindow("Color_image");
    return 0;
}

std::vector<cv::Point> find_bounding_box(std::vector<cv::Point>& pts)
{
    assert(pts.size() > 0); 
    cv::Point left_up, right_down; 
    left_up = right_down = pts[0]; 
    for(int j=1; j<pts.size();j++){
        cv::Point pt = pts[j]; 
        if(left_up.x > pt.x) left_up.x = pt.x; 
        if(left_up.y > pt.y) left_up.y = pt.y; 
        if(right_down.x < pt.x) right_down.x = pt.x; 
        if(right_down.y < pt.y) right_down.y = pt.y;
    }
    std::vector<cv::Point> bx; 
    bx.push_back(left_up); 
    bx.push_back(right_down); 
    return bx; 
}

std::vector<cv::Point> back_project(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pc_w, tf::Transform Tcw)
{
    std::vector<cv::Point> ret;

    for(int i=0; i<pc_w->points.size(); i++){

        pcl::PointXYZRGB& pt = pc_w->points[i]; 
        tf::Vector3 pt_w(pt.x, pt.y, pt.z); 
        tf::Vector3 pt_c = Tcw * pt_w; 

        // perspective projection 

        if(pt_c.getZ() < 0.02) continue; 

        cv::Point tmp_pt;
        tmp_pt.x = (pt_c.getX()/pt_c.getZ())*FX + CX; 
        tmp_pt.y = (pt_c.getY()/pt_c.getZ())*FY + CY; 

        if(tmp_pt.x <= 0 || tmp_pt.x >= IMAGE_WIDTH-1)
            continue; 
        if(tmp_pt.y <= 0 || tmp_pt.y >= IMAGE_HEIGHT -1)
            continue; 

        ret.push_back(tmp_pt); 
    }

    return ret; 
}

bool detect_stair(cv::Mat depth_image, std::vector<cv::Point>& bounding_box ){
    static int times = 0; 
    // static int times_threshold = 3; // consecutive times larger than this threshold 

    if(sdg.getStairs(depth_image, bounding_box)){
        ROS_INFO("stair_detection_pose_node: found potential stair, count %d", ++times); 
        if(times >= times_threshold){
            ROS_DEBUG("stair_detection_pose_node: succeed to detect stair"); 

            // try to extend the bounding box 
            cv::Point& left_up = bounding_box[0]; 
            cv::Point& right_down = bounding_box[1]; 

            // cout <<"before extend, "<<left_up.x<<" - "<<right_down.x<<endl; 
            int extend_len = 30;
            int left_most = left_up.x; 
            // int up_most = left_up.y; 
            int right_most = right_down.x; 
            // int down_most = right_down.y; 
            if(left_most - extend_len > 0)
                left_up.x = left_most - extend_len; 
            else
                left_up.x = 0; 
            if(right_most + extend_len < depth_image.cols)
                right_down.x = right_most + extend_len; 
            else
                right_down.x = depth_image.cols - 1; 

            // cout <<"after extend, "<<left_up.x<<" - "<<right_down.x<<endl; 

            return true; 
        }
        return false; 
    }else{
        times = 0; 
    }
    return false; 
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


