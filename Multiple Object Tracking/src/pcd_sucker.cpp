
#include <string>
#include <sstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
//#include <pcl/ros/conversions.h>
#include <velodyne_pointcloud/point_types.h>

#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>

using namespace std;
using namespace cv;

long int counter =0; 


ros::Subscriber sub_temp;

string stry;
string path;

const int alpha_slider_max = 100;
int alpha_slider;
double alpha;
double beta;

string WINDOW="features";

void callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
 pcl::PointCloud<pcl::PointXYZ> cloud;
 pcl::fromROSMsg(*laserCloudMsg, cloud);

 namedWindow(WINDOW,CV_WINDOW_AUTOSIZE); // Create Window
 createTrackbar("values",WINDOW, &alpha_slider, 100);
 alpha_slider = getTrackbarPos("values",WINDOW);

 counter++;
 cout<<"The value of alpha alpha_slider is "<<alpha_slider<<endl;
 ostringstream str1;
 str1 << counter;
 stry = str1.str();
 path = "test_pcd" + stry +".pcd";
 pcl::io::savePCDFileASCII (path, cloud);
 waitKey(1); 
}



int main (int argc, char** argv)
{
  ros::init(argc, argv,"pcd_sucker");
  ros::NodeHandle nh;
 // on_trackbar( alpha_slider, 0 );

  sub_temp= nh.subscribe("/clusterpointcloud", 1, callback);
  ros::spin();
  return 0;
}