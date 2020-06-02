#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#define PCL_NO_PRECOMPILE
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h>
using Eigen::MatrixXf;

using Eigen::MatrixXf;
using Eigen::VectorXf;
#define VPoint velodyne_pointcloud::PointXYZIR

// namespace clustering
// {
//   struct PointXYZIRL
//   {
//     PCL_ADD_POINT4D;                    
//     float    intensity;                 
//     uint16_t ring;                      
//     uint16_t label;                     
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
//   } EIGEN_ALIGN16;

// };
// POINT_CLOUD_REGISTER_POINT_STRUCT(clustering::PointXYZIRL,
//                                   (float, x, x)
//                                   (float, y, y)
//                                   (float, z, z)
//                                   (float, intensity, intensity)
//                                   (uint16_t, ring, ring)
//                                   (uint16_t, label, label))
// #define ClusterPointXYZIRL clustering::PointXYZIRL

pcl::PointCloud<VPoint>::Ptr combined_points(new pcl::PointCloud<VPoint>());

class velo_fusion{
public:
  velo_fusion();
private:
  ros::NodeHandle nh;
  ros::Subscriber cloud_f_sub;
  ros::Subscriber cloud_l_sub;
  ros::Subscriber cloud_r_sub;
  ros::Subscriber cloud_b_sub;
  ros::Publisher velodyne_combined_pub;

  void velodyne_callback_f(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void velodyne_callback_l(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void velodyne_callback_r(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void velodyne_callback_b(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);

  // point cloud shift to be imparted
  float fx = 0;
  float fy = 0;
  float fz = 0;
  float lx = 0;
  float ly = 0.41;
  float lz = 0;
  float rx = 0;
  float ry = -0.41;
  float rz = 0;
  float bx = -0.63;
  float by = 0;
  float bz = 0;  

};

velo_fusion::velo_fusion()
{
  cloud_f_sub = nh.subscribe("/vlp_1/velodyne_points", 2, &velo_fusion::velodyne_callback_b,this);
  cloud_l_sub = nh.subscribe("/vlp_2/velodyne_points", 2, &velo_fusion::velodyne_callback_l,this);
  // cloud_r_sub = nh.subscribe("/vlp_3/velodyne_points", 2, &velo_fusion::velodyne_callback_f,this);
  cloud_b_sub = nh.subscribe("/vlp_4/velodyne_points", 2, &velo_fusion::velodyne_callback_r,this);

  velodyne_combined_pub = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_combined",2);
}
void velo_fusion::velodyne_callback_f(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::cout<<"12 demons took my body"<<std::endl;  
  pcl::PointCloud<VPoint> laserCloudInput;
  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

  VPoint point;
  
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
      point.x = laserCloudInput.points[i].x + fx;
      point.y = laserCloudInput.points[i].y + fy;
      point.z = laserCloudInput.points[i].z + fz;
      point.intensity = laserCloudInput.points[i].intensity;
      point.ring = laserCloudInput.points[i].ring;
      combined_points->points.push_back(point);
  }
}

void velo_fusion::velodyne_callback_l(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::cout<<"Gotta Catch 'Em All"<<std::endl;
  pcl::PointCloud<VPoint> laserCloudInput;
  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

  VPoint point;
  
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
      point.x = laserCloudInput.points[i].x + lx;
      point.y = laserCloudInput.points[i].y + ly;
      point.z = laserCloudInput.points[i].z + lz;
      point.intensity = laserCloudInput.points[i].intensity;
      point.ring = laserCloudInput.points[i].ring + 15;
      combined_points->points.push_back(point);
  }
}

void velo_fusion::velodyne_callback_r(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::cout<<"The people of Ymir are the titans"<<std::endl;
  pcl::PointCloud<VPoint> laserCloudInput;
  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

  VPoint point;
  
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
      point.x = laserCloudInput.points[i].x + rx;
      point.y = laserCloudInput.points[i].y + ry;
      point.z = laserCloudInput.points[i].z + rz;
      point.intensity = laserCloudInput.points[i].intensity;
      point.ring = laserCloudInput.points[i].ring + 31;
      combined_points->points.push_back(point);
  }
}

void velo_fusion::velodyne_callback_b(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
{
  std::cout<<"My stand is a Secret"<<std::endl;
  pcl::PointCloud<VPoint> laserCloudInput;
  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);

  VPoint point;
  
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
      point.x = laserCloudInput.points[i].x + bx;
      point.y = laserCloudInput.points[i].y + by;
      point.z = laserCloudInput.points[i].z + bz;
      point.intensity = laserCloudInput.points[i].intensity;
      point.ring = laserCloudInput.points[i].ring + 47;
      combined_points->points.push_back(point);
  } 
  // Publish the points
  sensor_msgs::PointCloud2 fused_msg;
  pcl::toROSMsg(*combined_points,fused_msg);
  fused_msg.header.stamp = laserCloudMsg->header.stamp;
  fused_msg.header.frame_id = "velodyne";
  velodyne_combined_pub.publish(fused_msg);

  combined_points->clear();
}

int main(int argc, char **argv)
{
  std::cout<<"Griffith did nothing wrong"<<std::endl;
  ros::init(argc, argv,"velo_fusion_node");
  velo_fusion node;
  ros::spin();
  return 0;
}