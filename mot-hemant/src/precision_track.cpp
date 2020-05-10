#include <iostream>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include <time.h>
#include <velodyne_pointcloud/point_types.h>

namespace clustering
{
  struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    
    float    intensity;                 
    uint16_t ring;                      
    uint16_t label;                     
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;

};
POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label))

#define ClusterPointXYZIRL clustering::PointXYZIRL

using namespace std;

ros::Subscriber sub_clus;

// this function is for shifting the point cloud
void shift_cloud(pcl::PointCloud<ClusterPointXYZIRL>& poincloud, float x, float y){
	int npoints = poincloud.size();
	for(int i=0;i<npoints;i++){
		poincloud.points[i].x = poincloud.points[i].x - x;
		poincloud.points[i].y = poincloud.points[i].y - y;
	}
}

void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	clock_t tStart = clock();
 	pcl::PointCloud<ClusterPointXYZIRL> cluster;
    pcl::fromROSMsg(*cloud_msg, cluster);
    
    vector<pcl::PointCloud<pcl::PointXYZRGB> > clusters;

	int npoints = cluster.size();
	int nclusters = 1000;

	pcl::PointCloud<pcl::PointXYZRGB> tempy;
	for(int i=0;i<npoints;i++)
	{
		pcl::PointXYZRGB temp;
		temp.x = cluster.points[i].x;
		temp.y = cluster.points[i].y;
		temp.z = cluster.points[i].z;

		for(int j=0;j<nclusters;j++)
		{
			
		}
		tempy.points.push_back(temp);

	}

	tempy.width = tempy.size();
	tempy.height = 1;
	cout<<tempy<<endl;
	cout<<tempy.points[0].x<<endl;
	cout<<cluster<<endl;
}


int main(int argc, char** argv)
{
    // Initialize ROS
  ros::init (argc, argv,"posbox");
  ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/clusterpointcloud", 10, cloud_callback);
    
//   pub = nh.advertise<sensor_msgs::PointCloud2> ("just_clusters", 1);  
  
    ros::spin ();
}