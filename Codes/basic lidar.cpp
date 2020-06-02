kumawat.hemant 

#include <iostream>
#include <ros/ros.h>
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
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;


pcl::PointXYZIR iy ; // just a point 
iy.x 


pcl::PointCloud<pcl::PointXYZIR> ui ; //whole pointcloud
ui[i]x =9;

pcl::PointCloud<pcl::PointXYZIR>::Ptr hf //pointer to pointcloud


