#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "image_properties.h"
//ros libraries
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <fstream>
#include <algorithm>
//pcl libraries 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <velodyne_pointcloud/point_types.h>
#include <Eigen/Dense>
using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

using namespace cv;
using namespace std;
#define Vpoint velodyne_pointcloud::PointXYZIR

namespace mot_hemant{
	 struct PointXYZIRL{
	 	PCL_ADD_POINT4D;
	 	float intensity;
	 	int ring;
	 	int label;
	 	
	 }

}

ros::Subscriber velodyne_pointcloud;