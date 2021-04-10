
/*
  Mar. 31 2021, He Zhang, fuyinzh@gmail.com 

  generate point cloud from images 

*/

#include "pc_from_image.h"

// TODO: make these variables into config file 
double CX = 3.32695e+02; 
double CY = 2.58998e+02; 
double FX = 4.59357e+02; 
double FY = 4.59764e+02; 

int IMAGE_WIDTH = 640;
int IMAGE_HEIGHT = 480; 


int g_color[][3] = {255, 0, 0,  //RED
                            0, 255, 0, // GREEN
                            0, 0, 255,  // BLUE
                            255, 0, 255, // PURPLE
                            255, 255,255, // WHITE
                            255, 255, 0 // YELLOW
                        };

void generateColorPointCloud(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGB>& pc_out, int skip ) 
{  
  double z; 
  double px, py, pz; 
  int height = rgb.rows/skip; 
  int width = rgb.cols/skip; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 

  pc_out.points.reserve(N); 
  // pc.width = width; 
  // pc.height = height; 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 2, green_idx =1, blue_idx = 0;

  // Point pt; 
  pcl::PointXYZRGB pt;
  for(int v = 0; v<rgb.rows; v+=skip)
  for(int u = 0; u<rgb.cols; u+=skip)
  {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<unsigned short>((v), (u))*0.001;
    if(std::isnan(z) || z <= 0.0) 
    {
      // pt.x = std::numeric_limits<float>::quiet_NaN();  
      // pt.y = std::numeric_limits<float>::quiet_NaN();  
      // pt.z = std::numeric_limits<float>::quiet_NaN();  
      continue; 
    }

    double nu = (u - CX)/FX; 
    double nv = (v - CY)/FY; 
    px = nu * z; 
    py = nv * z; 
    pz = z; 

    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc_out.points.push_back(pt); 
  }
  pc_out.height = 1; 
  pc_out.width = pc_out.points.size(); 
  pc_out.is_dense = true; 
  return ;

}


void generateColorPointCloudBoundingBox(cv::Mat& rgb, cv::Mat& dpt, pcl::PointCloud<pcl::PointXYZRGB>& pc_out, std::vector<cv::Point>& bounding_box, int skip)
{
  double z; 
  double px, py, pz; 
  int height = rgb.rows/skip; 
  int width = rgb.cols/skip; 
  int N = (rgb.rows/skip)*(rgb.cols/skip); 

  pc_out.points.reserve(N); 
  // pc.width = width; 
  // pc.height = height; 

  unsigned char r, g, b; 
  int pixel_data_size = 3; 
  if(rgb.type() == CV_8UC1)
  {
    pixel_data_size = 1; 
  }
  
  int color_idx; 
  char red_idx = 2, green_idx =1, blue_idx = 0;

  // Point pt; 
  pcl::PointXYZRGB pt;

  cv::Point left_up = bounding_box[0]; 
  cv::Point right_down = bounding_box[1]; 

  int left_most = left_up.x; 
  int up_most = left_up.y; 
  int right_most = right_down.x; 
  int down_most = right_down.y; 

  for(int v = up_most; v<rgb.rows && v<down_most; v+=skip)
  for(int u = left_most; u<rgb.cols && u < right_most; u+=skip)
  {
    // Point& pt = pc.points[v*width + u]; 
    z = dpt.at<unsigned short>((v), (u))*0.001;
    if(std::isnan(z) || z <= 0.0) 
    {
      // pt.x = std::numeric_limits<float>::quiet_NaN();  
      // pt.y = std::numeric_limits<float>::quiet_NaN();  
      // pt.z = std::numeric_limits<float>::quiet_NaN();  
      continue; 
    }

    double nu = (u - CX)/FX; 
    double nv = (v - CY)/FY; 
    px = nu * z; 
    py = nv * z; 
    pz = z; 

    pt.x = px;  pt.y = py;  pt.z = pz; 
    color_idx = (v*rgb.cols + u)*pixel_data_size;
    if(pixel_data_size == 3)
    {
      r = rgb.at<uint8_t>(color_idx + red_idx);
      g = rgb.at<uint8_t>(color_idx + green_idx); 
      b = rgb.at<uint8_t>(color_idx + blue_idx);
    }else{
      r = g = b = rgb.at<uint8_t>(color_idx); 
    }
    pt.r = r; pt.g = g; pt.b = b; 
    pc_out.points.push_back(pt); 
  }
  pc_out.height = 1; 
  pc_out.width = pc_out.points.size(); 
  pc_out.is_dense = true; 
  return ;
} 



void generateFilteredPointCloud(const cv::Mat& dpt_img, pcl::PointCloud<pcl::PointXYZ>& pc_out)
{
    // median filter to get rid some noise 
    // cv::Mat dpt_img = cv_bridge::toCvCopy(dpt_img)->image;
    // cv::Mat dst; 
    // cv::medianBlur(dpt_img, dst, 5);  
    // dpt_img = dst; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr tmpPC(new pcl::PointCloud<pcl::PointXYZ>); 
    double cloud_dense_rate = 5; 
    double halfDS = cloud_dense_rate/2. - 0.5; 
    float scale = 0.001; 
    float min_dis = 0.3; 
    float max_dis = 7.0; // mMaxDepth;  // keep depth range 
    for(double i = halfDS; i < dpt_img.rows; i += cloud_dense_rate)
    for(double j = halfDS; j < dpt_img.cols; j += cloud_dense_rate)
    {
        int pixelCnt = 0; 
        float vd, vd_sum = 0; 
        int is = (int)(i - halfDS); int ie = (int)(i + halfDS); 
        int js = (int)(j - halfDS); int je = (int)(j + halfDS);
        for(int ii = is; ii<= ie; ii++)
        for(int jj = js; jj<= je; jj++)
        {
            unsigned short _dpt = dpt_img.at<unsigned short>(ii, jj); 
            vd = _dpt * scale; 
            // vd = syncCloud2Pointer[ii * dpt_img.cols + jj]; 
            if(vd > min_dis && vd < max_dis)
            {
            pixelCnt++; 
            vd_sum += vd; 
            }
        }
        if(pixelCnt > 0)
        {
            double u = (j - CX)/FX;
            double v = (i - CY)/FY; 
            double mean_vd = vd_sum / pixelCnt; 
            pcl::PointXYZ pt;
            pt.x = u * mean_vd; 
            pt.y = v * mean_vd;
            pt.z = mean_vd; 
            // pt.intensity = 1; // timeElapsed;
            tmpPC->points.push_back(pt); 
        }
    }

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    downSizeFilter.setInputCloud(tmpPC);
    downSizeFilter.setLeafSize(0.03, 0.03, 0.03);
    downSizeFilter.filter(pc_out);
    return ; 
}


bool markColor(pcl::PointCloud<pcl::PointXYZRGB>& cloud, COLOR c)
{
  if(cloud.points.size() <= 0)
  {
    // cout<<"global_def.cpp: cloud has no points!"<<endl;
    return false;
  }
  int N = cloud.points.size();
  for(int i=0; i<N; i++)
  {
    cloud.points[i].r = g_color[c][0];
    cloud.points[i].g = g_color[c][1];
    cloud.points[i].b = g_color[c][2];
  }
  return true;
}