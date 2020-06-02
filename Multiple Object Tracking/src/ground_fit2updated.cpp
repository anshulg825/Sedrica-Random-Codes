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

#include <Eigen/Dense>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <math.h>
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
//std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(100);
pcl::PointCloud<VPoint>::Ptr g_ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr g_not_ground_points(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr ggroundpoints(new pcl::PointCloud<VPoint>());
pcl::PointCloud<VPoint>::Ptr gnotgroundpoints(new pcl::PointCloud<VPoint>());
pcl::PointCloud<ClusterPointXYZIRL>::Ptr g_all_pc(new pcl::PointCloud<ClusterPointXYZIRL>());

bool x_comparison(VPoint a , VPoint b){
	return a.x<b.x;
};

class plane_fit{
public:
	plane_fit();
private:
	ros::NodeHandle nh;
	ros::Subscriber points_sub;
	ros::Publisher ground_points_pub;
	ros::Publisher all_points_pub;
	ros::Publisher groudless_points_pub;

	// Parameters for sensors and plane fit 
	//CHeck once with papere

	
	int al = HUGE_VAL;//need to set threshold
	std::vector< pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > sourceClouds;

	void velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg);
	void make_rings(const pcl::PointCloud<VPoint>& laserCloudInput);
	void find_boundary_points(std::vector< pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > &sourceClouds );
	float cos_rule(VPoint point1, VPoint point2, VPoint point3);
	void copyPointCloud( const pcl::PointCloud<VPoint>::Ptr &model_normals ,const pcl::PointCloud<VPoint> input_cloud){

    //Compute normals  
	 	*model_normals=input_cloud;
    }
};

plane_fit::plane_fit(){
		points_sub = nh.subscribe("/velodyne_points", 2, &plane_fit::velodyne_callback,this);
		groudless_points_pub = nh.advertise<sensor_msgs::PointCloud2>("/points_no_ground",2);
		ground_points_pub= nh.advertise<sensor_msgs::PointCloud2>("/points_ground",2);
		all_points_pub=nh.advertise<sensor_msgs::PointCloud2>("/all_points",2);

	}


//This function creates a vector which contains point clouds according to rings	
void plane_fit::make_rings(const pcl::PointCloud<VPoint>& laserCloudInput){
	for(int i=0;i<64;i++){
        pcl::PointCloud<VPoint>::Ptr Rings(new pcl::PointCloud<VPoint>);
		sourceClouds.push_back(Rings);
		}
	for(size_t i=0;i<laserCloudInput.points.size();i++){
		int j= laserCloudInput.points[i].ring;
		sourceClouds[j]->points.push_back(laserCloudInput.points[i]);
	}
}

//This function gives us final ground points of individual rings
void plane_fit::find_boundary_points(std::vector< pcl::PointCloud<VPoint>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <VPoint>::Ptr > > &sourceClouds )
{	
					std::cout << "Bring me thanos in 1" << std::endl;
	for(int i=0;i<64;i++){
					std::cout << "Bring me thanos in 2" << std::endl;

		for(int j=0;j<sourceClouds[i]->points.size();j++){
					std::cout << "Bring me thanos in 3" << std::endl;

			sort(sourceClouds[i]->points.begin(),sourceClouds[i]->points.end(),x_comparison);
			if(int(sourceClouds[i]->points[j].x)==0){ //TODO change int
					std::cout << "Bring me thanos in 4" << std::endl;

				for(int k=j-1;k>0;k--){
					std::cout << "Bring me thanos in 5" << std::endl;

					float Alpha = cos_rule(sourceClouds[i]->points[k-1],sourceClouds[i]->points[k],sourceClouds[i]->points[k+1]);
					ite--;
					if (Alpha<al){
					std::cout << "Bring me thanos in 6" << std::endl;

						//ClusterPointXYZIRL point;
						for(size_t l=0;l<k+1;l++){
	        				gnotgroundpoints->points.push_back(sourceClouds[i]->points[l]);
					std::cout << "Bring me thanos in 7" << std::endl;

		                }
		                for(size_t l=k+1;l<j+1;l++){
							ggroundpoints->points.push_back(sourceClouds[i]->points[l]);
					std::cout << "Bring me thanos in 8" << std::endl;

		                }

					}
					std::cout << "Bring me thanos" << std::endl;
					std::cout << "Bring me thanos alpha" << Alpha << std::endl;

				break;
				}

				for(int k=j+1;k<sourceClouds[i]->points.size()-1;k++){
					std::cout << "Bring me thanos in for" << std::endl;

					float Alpha = cos_rule(sourceClouds[i]->points[k-1],sourceClouds[i]->points[k],sourceClouds[i]->points[k+1]);
					if (Alpha<al){
						for(size_t l=k;l<sourceClouds[i]->points.size();l++){
	        				gnotgroundpoints->points.push_back(sourceClouds[i]->points[l]);
		                }
		                for(size_t l=j;l<k;l++){
							ggroundpoints->points.push_back(sourceClouds[i]->points[l]);
							std::cout << "in loop" << l << std::endl;
		                }
						
					}
					break;
				}
			}
		}
}	}	

//this function applies cosine rule
 float plane_fit::cos_rule(VPoint point1, VPoint point2, VPoint point3)
 {
 	float D12 = sqrt(pow(point2.x-point1.x , 2) + pow(point2.y-point1.y , 2) + pow(point2.z-point1.z , 2));
	float D23 = sqrt(pow(point3.x-point2.x , 2) + pow(point3.y-point2.y , 2) + pow(point3.z-point2.z , 2));
	float D31 = sqrt(pow(point3.x-point1.x , 2) + pow(point3.y-point1.y , 2) + pow(point3.z-point1.z , 2));
	if ( ((pow(D12,2) + pow(D23,2) - pow(D31,2))/ 2*D12*D23) >1  || ((pow(D12,2) + pow(D23,2) - pow(D31,2))/ 2*D12*D23) <(-1))
	std::cout << "Bring me ruchika in function" << std::endl;

	float Alpha = acos( (pow(D12,2) + pow(D23,2) - pow(D31,2))/ 2*D12*D23);
  	return Alpha;
  }

 void plane_fit::velodyne_callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
	pcl::PointCloud<VPoint> laserCloudInput;
	pcl::fromROSMsg(*laserCloudMsg, laserCloudInput);
	make_rings(laserCloudInput);
	find_boundary_points(sourceClouds);
	ClusterPointXYZIRL point;
    	for(size_t i=0;i<ggroundpoints->points.size();i++){
	        point.x = ggroundpoints->points[i].x;
	        point.y = ggroundpoints->points[i].y;
	        point.z = ggroundpoints->points[i].z;
	        point.intensity = ggroundpoints->points[i].intensity;
	        point.ring = ggroundpoints->points[i].ring;
	        point.label = 1u;// 0 means uncluster
	        g_all_pc->points.push_back(point);
	    }
	    for(size_t i=0;i<gnotgroundpoints->points.size();i++){
	        point.x = gnotgroundpoints->points[i].x;
	        point.y = gnotgroundpoints->points[i].y;
	        point.z = gnotgroundpoints->points[i].z;
	        point.intensity = gnotgroundpoints->points[i].intensity;
	        point.ring = gnotgroundpoints->points[i].ring;
	        point.label = 0u;// 0 means uncluster
	        g_all_pc->points.push_back(point);
	    }
	// g_ground_points = ggroundpoints;
	// g_not_ground_points = gnotgroundpoints;
	
	sensor_msgs::PointCloud2 ground_msg;
		pcl::toROSMsg(*ggroundpoints,ground_msg);
		ground_msg.header.stamp = laserCloudMsg->header.stamp;
    	ground_msg.header.frame_id = laserCloudMsg->header.frame_id;
    	ground_points_pub.publish(ground_msg);

    	sensor_msgs::PointCloud2 groundless_msg;
	    pcl::toROSMsg(*gnotgroundpoints, groundless_msg);
	    groundless_msg.header.stamp = laserCloudMsg->header.stamp;
	    groundless_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    groudless_points_pub.publish(groundless_msg);
	    
	    sensor_msgs::PointCloud2 all_points_msg;
	    pcl::toROSMsg(*g_all_pc, all_points_msg);

	    all_points_msg.header.stamp = laserCloudMsg->header.stamp;
	    all_points_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    all_points_pub.publish(all_points_msg);
	    g_all_pc->clear();
	    std::cout<<"ahfvsdlbwe\n";


}


int main(int argc, char **argv)
{

    ros::init(argc, argv,"plane_fit_node");
    plane_fit node;
    ros::spin();
    ros::Rate loop_rate(10);
    loop_rate.sleep();

    return 0;

}