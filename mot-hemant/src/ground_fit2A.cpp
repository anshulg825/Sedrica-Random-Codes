#include <iostream>
//#include <home/sine/Downloads/pcl-pcl-1.8.0/gpu/features/include/pcl/gpu/features/features.hpp>
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
#include <algorithm>
#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <typeinfo>       // operator typeid

using Eigen::MatrixXf;
using Eigen::VectorXf;
#define VPoint velodyne_pointcloud::PointXYZIR

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
std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > RingClouds(64);
pcl::PointCloud<VPoint>::Ptr ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr road_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr lane_points(new pcl::PointCloud<VPoint>());
//pcl::PointCloud<VPoint>::Ptr g_not_ground_points(new pcl::PointCloud<VPoint>());
bool y_comparison(VPoint a , VPoint b){
  return a.y<b.y;
};

class road3d{
public:
  road3d();
private:
  ros::NodeHandle nh;
  ros::Subscriber points_sub;
  ros::Publisher road_points_pub;
  ros::Publisher lane_points_pub;

  // Parameters for sensors and plane fit 
  //CHeck once with papere

  
  double sensor_height = 3;
  int seg_no = 1; /// to be  implemented after tuning 
  int iter_no = 4;
  int lpr_no = 20;
  double th_seeds = 1.2;
  double th_dist = 0.5;
  void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
  void road_extractor(const pcl::PointCloud<VPoint>::Ptr &RingClouds);

    //Compute normals  
  //   *model_normals=input_cloud;
  // }
  int t;
  float threshold_ang=0.2;
  float d; /// eqation parameter 
  // Normal  parameters
  MatrixXf normal; // current size 0x0;

  float th_dist_d;

};

road3d::road3d()
{
  points_sub = nh.subscribe("/points_ground", 2, &road3d::velodyne_callback,this);

  road_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_road",2);
  lane_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_lane",2);
}

void road3d::road_extractor(const pcl::PointCloud<VPoint>::Ptr &RingClouds)
{ 
	std::cout<<"function properly call, fingers crossed"<<std::endl;
  bool s=true;
  for(int i=0;i<RingClouds->points.size() && s;i++){
    std::cout<<"ok"<<std::endl;
  if(RingClouds->points[i].y>-0.005 && RingClouds->points[i].y<0.005){
  std::cout<<"ok1"<<std::endl;  
  t=i;
  s=false;
  }
}
  for(size_t j=t-1;j<RingClouds->points.size() && RingClouds->points[j].x>0;j++){
    std::cout<<"ok1"<<std::endl; 
    float D12 = sqrt(pow(RingClouds->points[j+1].x- RingClouds->points[j].x, 2) + pow(RingClouds->points[j+1].y-RingClouds->points[j].y, 2) + pow(RingClouds->points[j+1].z-RingClouds->points[j].z, 2));
    float D23 = sqrt(pow(RingClouds->points[j+2].x-RingClouds->points[j+1].x , 2) + pow(RingClouds->points[j+2].y-RingClouds->points[j+1].y , 2) + pow(RingClouds->points[j+2].z-RingClouds->points[j+1].z , 2));
    float D31 = sqrt(pow(RingClouds->points[j+2].x-RingClouds->points[j].x , 2) + pow(RingClouds->points[j+2].y-RingClouds->points[j].y , 2) + pow(RingClouds->points[j+2].z-RingClouds->points[j].z , 2));
    float Alpha = acos( (pow(D12,2) + pow(D23,2) - pow(D31,2))/ (2*D12*D23));
    std::cout<<Alpha<<std::endl; 
    if(Alpha<threshold_ang){
      std::cout<<"ok2"<<std::endl; 
      road_points->points.push_back(RingClouds->points[j]);
    }
    else
      break;
  }
  for(size_t j=t-2;j>0 && RingClouds->points[j].x>0;j--){
    std::cout<<"ok3"<<std::endl; 
    float D12 = sqrt(pow(RingClouds->points[j+1].x- RingClouds->points[j].x, 2) + pow(RingClouds->points[j+1].y-RingClouds->points[j].y, 2) + pow(RingClouds->points[j+1].z-RingClouds->points[j].z, 2));
    float D23 = sqrt(pow(RingClouds->points[j+2].x-RingClouds->points[j+1].x , 2) + pow(RingClouds->points[j+2].y-RingClouds->points[j+1].y , 2) + pow(RingClouds->points[j+2].z-RingClouds->points[j+1].z , 2));
    float D31 = sqrt(pow(RingClouds->points[j+2].x-RingClouds->points[j].x , 2) + pow(RingClouds->points[j+2].y-RingClouds->points[j].y , 2) + pow(RingClouds->points[j+2].z-RingClouds->points[j].z , 2));
    float Alpha = acos( (pow(D12,2) + pow(D23,2) - pow(D31,2))/ (2*D12*D23));
    std::cout<<Alpha<<std::endl; 
    if(Alpha<threshold_ang){
      std::cout<<"ok1"<<std::endl; 
      road_points->points.push_back(RingClouds->points[j]);
    }
    else
      break;
    std::cout<<"BOOM goes the World"<<std::endl;
  }
    
}

void road3d::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& ground_msg)
{

  std::cout<<"atleast call back works"<<std::endl;
  pcl::PointCloud<VPoint>::Ptr model_normals (new pcl::PointCloud<VPoint> ());

  pcl::PointCloud<VPoint> laserCloudInput;

  // conversion of velodyne point struct to pcl data point 
  pcl::fromROSMsg(*ground_msg, laserCloudInput);
  // std::vector < pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > RingClouds;
  VPoint point;
  // point.x 
  std::cout<<"I am your father, Luke"<<std::endl;
  for(size_t i=0;i<laserCloudInput.points.size();i++)
  {
  	std::cout<<"I will kill you, Sreeraj"<<std::endl;
    point.x = laserCloudInput.points[i].x;
    point.y = laserCloudInput.points[i].y;
    point.z = laserCloudInput.points[i].z;
    point.intensity = laserCloudInput.points[i].intensity;
    point.ring = laserCloudInput.points[i].ring;
    std::cout<<"Pranav will go to prison"<<std::endl;
    ground_points->points.push_back(point);
    std::cout<<"Ruchika is gonna be pushed_back"<<std::endl;
    int xvb = 461;
    std::cout << typeid(point).name() << '\n';
    std::cout << typeid(xvb).name() << '\n';
    std::cout << typeid(RingClouds).name() << '\n';
    std::cout<<point.ring<<std::endl;
    RingClouds[point.ring]->points.push_back(point);
    std::cout<<RingClouds[point.ring]->points.size()<<std::endl;
    // RingClouds.push_back(point);

    std::cout<<"Just leave it [Blank]"<<std::endl;
  }
  std::cout<<"about to call the road road_extractor"<<std::endl;
  for(int i = 0; i<64; i++)
  {
    sort(RingClouds[i]->points.begin(), RingClouds[i]->points.end(),y_comparison);
    road_extractor(RingClouds[i]);
  }

  // Publish the fucking points

  sensor_msgs::PointCloud2 road_msg;
  pcl::toROSMsg(*road_points,road_msg);
  road_msg.header.stamp = ground_msg->header.stamp;
  road_msg.header.frame_id = ground_msg->header.frame_id;
  road_points_pub.publish(road_msg);
  std::cout<<"I am Inevitable"<<std::endl;
}

int main(int argc, char **argv)
{
	std::cout<<"The Wands true owner wasn't Snape"<<std::endl;
  ros::init(argc, argv,"road3d_node");
  road3d node;
  ros::spin();
  return 0;
}