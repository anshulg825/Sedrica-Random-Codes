
// Motion Estimation
/// I am sadddddd and disappointed
//// Why am I talking to you??     
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
//#include <pcl/ros/conversions.h>
#include <velodyne_pointcloud/point_types.h>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/common/io.h>

using Eigen::MatrixXf;
using Eigen::JacobiSVD;
using Eigen::VectorXf;

#define VPoint velodyne_pointcloud::PointXYZIR
#define Geometric_consistency 
typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;


int no_of_clusters;

float distance_origin(pcl::PointXYZ point_a){
  return sqrt((point_a.x)*(point_a.x) + (point_a.y)*(point_a.y) + (point_a.z)*(point_a.z) );
}

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

/*namespace motionEstimator
{
  struct PointXYZIRLTS
  {
    PCL_ADD_POINT4D;                    
    float    intensity;                 
    uint16_t ring;                      
    uint16_t label;
    uint16_t track_id;
    uint16_t shot;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
  } EIGEN_ALIGN16;

};
POINT_CLOUD_REGISTER_POINT_STRUCT(motionEstimator::PointXYZIRLTS,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (uint16_t, label, label)
                                  (uint16_t track_id, track_id)
                                  (uint16_t shot))*/


#define ClusterPointXYZIRL clustering::PointXYZIRL
pcl::PointCloud<ClusterPointXYZIRL> laserCloudIn_old;
int no_of_clusters_old;

visualization_msgs::Marker track_box(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, 
    std::string ns ,int id, float r, float g, float b) { 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (*cloud_cluster, centroid); 
  pcl::getMinMax3D (*cloud_cluster, min, max); 
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 

  visualization_msgs::Marker marker; 
  marker.header.frame_id = "/velodyne"; 
  marker.header.stamp = ros::Time::now(); 
  std::string s = std::to_string(id);
  //marker.text= s;
  marker.ns = ns; 
  marker.id = id; 
  marker.type = shape; 
  marker.action = visualization_msgs::Marker::ADD; 
  
  marker.pose.position.x = centroid[0]; 
  marker.pose.position.y = centroid[1]; 
  marker.pose.position.z = centroid[2]; 
  marker.pose.orientation.x = 0.0; 
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0; 
  marker.pose.orientation.w = 1.0; 
  
  marker.scale.x = (max[0]-min[0]); 
  marker.scale.y = (max[1]-min[1]); 
  marker.scale.z = (max[2]-min[2]); 
  
  if (marker.scale.x ==0) 
      marker.scale.x=0.1; 

  if (marker.scale.y ==0) 
    marker.scale.y=0.1; 

  if (marker.scale.z ==0) 
    marker.scale.z=0.1; 
    
  marker.color.r = r; 
  marker.color.g = g; 
  marker.color.b = b; 
  marker.color.a = 1; 

  marker.lifetime = ros::Duration(0.1); 

  return marker; 
}

visualization_msgs::Marker track_id(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster, 
    std::string ns ,int id, float r, float g, float b) { 
  Eigen::Vector4f centroid; 
  Eigen::Vector4f min; 
  Eigen::Vector4f max; 
  
  pcl::compute3DCentroid (*cloud_cluster, centroid); 
  pcl::getMinMax3D (*cloud_cluster, min, max); 
  
  uint32_t shape = visualization_msgs::Marker::CUBE; 

  visualization_msgs::Marker text_id; 
  text_id.header.frame_id = "/velodyne"; 
  text_id.header.stamp = ros::Time::now(); 
  std::string s = std::to_string(id);
  text_id.text= s;
  text_id.ns = ns; 
  text_id.id = id; 
  text_id.type = shape; 
  text_id.action = visualization_msgs::Marker::ADD; 
  text_id.pose.position.x = centroid[0]; 
  text_id.pose.position.y = centroid[1]; 
  text_id.pose.position.z = centroid[2]; 
  text_id.pose.orientation.x = 0.0; 
  text_id.pose.orientation.y = 0.0;
  text_id.pose.orientation.z = 0.0; 
  text_id.pose.orientation.w = 1.0; 
  
  text_id.scale.x = (max[0]-min[0]); 
  text_id.scale.y = (max[1]-min[1]); 
  text_id.scale.z = (max[2]-min[2]); 
  
  if (text_id.scale.x ==0) 
      text_id.scale.x=0.1; 

  if (text_id.scale.y ==0) 
    text_id.scale.y=0.1; 

  if (text_id.scale.z ==0) 
    text_id.scale.z=0.1; 
    
  text_id.color.r = r; 
  text_id.color.g = g; 
  text_id.color.b = b; 
  text_id.color.a = 1; 

  text_id.lifetime = ros::Duration(0.1); 

  return text_id; 
}

bool contains(pcl::PointCloud<pcl::PointXYZ>::Ptr c, pcl::PointXYZ p) {
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c->begin();
    for (; it != c->end(); ++it) {
        if (it->x == p.x && it->y == p.y && it->z == p.z)
            return true;
    }
    return false;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr intersection(pcl::PointCloud<pcl::PointXYZ>::Ptr c1,
        pcl::PointCloud<pcl::PointXYZ>::Ptr c2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr inter;
    pcl::PointCloud<pcl::PointXYZ>::iterator it = c1->begin();
    for (; it != c1->end(); ++it) {
        if (contains(c2, *it))
            inter->push_back(*it);
    }

    return inter;
}
#define ClusterPointXYZIRL clustering::PointXYZIRL

bool label_comparison(ClusterPointXYZIRL a , ClusterPointXYZIRL b){
  return a.label<b.label;
};

class MotionEstimation{
public:
	MotionEstimation(){
		 velodyneFrame_sub = nh.subscribe("clusterpointcloud", 2, &MotionEstimation::callback, this);
		 marker_array_id_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/cluster_ma_id", 10);
		 marker_array_box_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/cluster_ma_box", 10);
	}
private:
	ros::NodeHandle nh;
	ros::Subscriber velodyneFrame_sub;
	ros::Publisher marker_array_id_pub_;
	ros::Publisher marker_array_box_pub_;


	int sensor_model = 32;

	  	pcl::PointCloud<pcl::PointXYZ>::Ptr extractSphereradii(pcl::PointXYZ searchPoint,float radius,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (cloud);
		std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
		std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding points

		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
		//     for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
		//         std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
		//                 << " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
		//                 << " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
		//                 << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		// }
  	}

  		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
		    cloud_cluster->points.push_back(cloud->points[ pointIdxRadiusSearch[i] ]);}
			cloud_cluster->width = cloud_cluster->points.size ();
			cloud_cluster->height = 1;
			cloud_cluster->is_dense = true;
		return cloud_cluster;
  }


	void callback(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){
	visualization_msgs::MarkerArray ma_id;
	visualization_msgs::MarkerArray ma_box;
	std::vector<pcl::Correspondences> clustered_corrs;
    
    pcl::PointCloud<ClusterPointXYZIRL> laserCloudIn;
    pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);
    ////// sort the cloud based on labels 
    sort(laserCloudIn.points.begin(), laserCloudIn.points.end(),label_comparison);
    pcl::PointCloud<ClusterPointXYZIRL>::iterator it_end = laserCloudIn.points.end();
    it_end = it_end-1;
    no_of_clusters = it_end->label;
    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds(no_of_clusters);

    /////Cloud converted to PointXYZ and stored in vector 
    for(int i=0;i<laserCloudIn.points.size();i++){
      int label_t = laserCloudIn.points[i].label;
      pcl::PointXYZ dummy_point;
      dummy_point.x=laserCloudIn.points[i].x;
      dummy_point.y=laserCloudIn.points[i].y;
      dummy_point.z=laserCloudIn.points[i].z;
      sourceClouds[label_t]->points.push_back(dummy_point);

    }


    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > >::iterator cloudIterator ;


    for(cloudIterator=sourceClouds.begin();cloudIterator<sourceClouds.end();cloudIterator++){
      if((*cloudIterator)->points.size()==0) sourceClouds.erase(cloudIterator);
    }
    no_of_clusters = sourceClouds.size();
    std::vector<Eigen::Vector4f > centroid(no_of_clusters) ;
    std::vector<int> label_no(no_of_clusters);

    for(int i=0;i<sourceClouds.size();i++){
      pcl::compute3DCentroid (*sourceClouds[i], centroid[i]);

    }


    std::vector< pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds_old(no_of_clusters_old);

    /////Cloud converted to PointXYZ and stored in vector 
    for(int i=0;i<laserCloudIn_old.points.size();i++){
      int label_t = laserCloudIn_old.points[i].label;
      pcl::PointXYZ dummy_point;
      dummy_point.x=laserCloudIn.points[i].x;
      dummy_point.y=laserCloudIn.points[i].y;
      dummy_point.z=laserCloudIn.points[i].z;
      sourceClouds_old[label_t]->points.push_back(dummy_point);

    }



    for(cloudIterator=sourceClouds_old.begin();cloudIterator<sourceClouds_old.end();cloudIterator++){
      if((*cloudIterator)->points.size()==0) sourceClouds_old.erase(cloudIterator);
    }
    no_of_clusters_old = sourceClouds_old.size();
    std::vector<Eigen::Vector4f > centroid_old(no_of_clusters_old) ;
    std::vector<int> label_no_old(no_of_clusters_old);

    for(int i=0;i<sourceClouds_old.size();i++){
      pcl::compute3DCentroid (*sourceClouds_old[i], centroid_old[i]);


    }

    if(laserCloudIn_old.points.size()==0) {
    	laserCloudIn_old = laserCloudIn_old;
    	return;
    }

    for(int i=0;i<no_of_clusters_old ;i++){
    	pcl::PointXYZ point;
    	point.x=centroid_old[i][0];
    	point.x=centroid_old[i][1];
    	point.x=centroid_old[i][2];
    	pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudIn_XYZ (new pcl::PointCloud<pcl::PointXYZ>);
    	pcl::copyPointCloud(laserCloudIn,*laserCloudIn_XYZ);
      std::cout<<laserCloudIn_XYZ->point.size();
    	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud (new pcl::PointCloud<pcl::PointXYZ>) = extractSphereradii(point, 2.0f,laserCloudIn_XYZ);
    	//		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud = *scene_cloud_ptr;
    	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZ>)= sourceClouds_old[i]; 
  		pcl::PointCloud<pcl::Normal>::Ptr normal_sourceCloud (new pcl::PointCloud<NormalType> ()) = normalEstimation(source_cloud);
  		pcl::PointCloud<pcl::Normal>::Ptr normal_sceneCloud (new pcl::PointCloud<NormalType> ())= normalEstimation(scene_cloud);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_sourceCloud (new pcl::PointCloud<pcl::PointXYZ>) = keypointsEstimation(source_cloud,normal_sourceCloud);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_sceneCloud(new pcl::PointCloud<pcl::PointXYZ>) = keypointsEstimation(scene_cloud,normal_sceneCloud);
  		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_sourceCloud (new pcl::PointCloud<DescriptorType> ())= descriptorsEstimation(source_cloud , normal_sourceCloud ,keypoints_sourceCloud);
  		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_sceneCloud (new pcl::PointCloud<DescriptorType> ()) = descriptorsEstimation(scene_cloud , normal_sceneCloud ,keypoints_sceneCloud);
  		pcl::CorrespondencesPtr model_corrs (new pcl::Correspondences ())= correspondence_computation(descriptor_sourceCloud,descriptor_sceneCloud);
  		std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > motion_model=motionMatrixes(source_cloud,keypoints_sourceCloud,normal_sourceCloud,scene_cloud,keypoints_sceneCloud,normal_sceneCloud,model_corrs,clustered_corrs);
  		if(model_corrs->size()==0) {std:;cout<<label_no_old[i]<<" lost tarack of id "<<label_no_old[i]<<" "<<endl; continue;}

  		pcl::PointCloud<pcl::PointXYZ>::Ptr model_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_pointCloud(new pcl::PointCloud<pcl::PointXYZ>);

  		for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
	     {
	        std::stringstream ss_line;
	        ss_line << "correspondence_line" << i << "_" << j;
	        pcl::PointXYZ& model_point = keypoints_sourceCloud->at (clustered_corrs[i][j].index_query);
	        pcl::PointXYZ& scene_point = keypoints_sceneCloud->at (clustered_corrs[i][j].index_match);

	        //  We are drawing a line for each pair of clustered correspondences found between the model and the scene
	        model_pointCloud->points.push_back(model_point);
	        scene_pointCloud->points.push_back(scene_point);
	        


	     }

      float col = i/no_of_clusters_old;
      ma_box.markers.push_back(track_box(scene_pointCloud, "box" ,i, col,  col, col));
      ma_id.markers.push_back(track_id(scene_pointCloud, "box_id" ,i, col, col, col));



  		




    }


       marker_array_id_pub_.publish(ma_id);
       marker_array_box_pub_.publish(ma_box);







  }



	

/*  	pcl::CorrespondencesPtr correspondence_matching( const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud){
  		pcl::PointCloud<pcl::Normal>::Ptr normal_sourceCloud = normalEstimation(source_cloud);
  		pcl::PointCloud<pcl::Normal>::Ptr normal_sceneCloud = normalEstimation(scene_cloud);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_sourceCloud = keypointsEstimation(source_cloud,normal_sourceCloud);
  		pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_sceneCloud = keypointsEstimation(scene_cloud,normal_sceneCloud);
  		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_sourceCloud = descriptorsEstimation(source_cloud , normal_sourceCloud ,keypoints_sourceCloud);
  		pcl::PointCloud<pcl::SHOT352>::Ptr descriptor_sceneCloud = descriptorsEstimation(scene_cloud , normal_sceneCloud ,keypoints_sceneCloud);
  		pcl::CorrespondencesPtr model_corrs = correspondence_computation(descriptor_sourceCloud,descriptor_sceneCloud);
  		return model_corrs;

  	}
*/
	//pcl::PointCloud<pcl::SHOT352>::Ptr shotestimation( const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
  pcl::PointCloud<pcl::Normal>::Ptr normalEstimation( const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){

    //Compute normals  
    pcl::PointCloud<pcl::Normal>::Ptr model_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setKSearch (10);
    norm_est.setInputCloud (input_cloud);
    norm_est.compute (*model_normals);
    return model_normals;}



    //  Downsample Clouds to Extract keypoints
  pcl::PointCloud<pcl::PointXYZ>::Ptr keypointsEstimation(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, const pcl::PointCloud<pcl::Normal>::Ptr model_normals){
    pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints (new pcl::PointCloud<pcl::PointXYZ> ());
    float model_ss_ (0.01f);
    pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
    uniform_sampling.setInputCloud (input_cloud);
    uniform_sampling.setRadiusSearch (model_ss_); // Parameter to tune 
    uniform_sampling.filter (*model_keypoints);
    std::cout << "Model total points: " << input_cloud->size () << "; Selected Keypoints: " << model_keypoints->size () << std::endl;
    return model_keypoints;
  }

    //Compute descripters 
  pcl::PointCloud<pcl::SHOT352>::Ptr descriptorsEstimation( const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, const pcl::PointCloud<pcl::Normal>::Ptr model_normals, const pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints){ 
    float descr_rad_ (0.02f);
    pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
    pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352> ());
    descr_est.setRadiusSearch (descr_rad_);

    descr_est.setInputCloud (model_keypoints);
    descr_est.setInputNormals (model_normals);
    descr_est.setSearchSurface (input_cloud);
    descr_est.compute (*model_descriptors);

    return model_descriptors;
  }


	
  pcl::CorrespondencesPtr correspondence_computation(const pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors, const pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors){
  	pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
  	pcl::KdTreeFLANN<pcl::SHOT352> match_search;
  	match_search.setInputCloud (model_descriptors);
  	for(size_t i =0;i< scene_descriptors->size();++i){
  		std::vector<int> neigh_indices(1);
  		std::vector<float> neigh_sqr_dists(1);
  		if (!pcl_isfinite (scene_descriptors->at (i).descriptor[0])) //skipping NaNs
	    {
	      continue; //////// Told you I will be here forever 
	      /// You can stand my umbrella ells ellla eh eh eh 
        /// Oh baby it's raining , raining 
	    }
	    int found_neighs = match_search.nearestKSearch (scene_descriptors->at (i), 1, neigh_indices, neigh_sqr_dists);
	    if(found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
	    {
	      pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
	      model_scene_corrs->push_back (corr);
	    }
  	}
    return model_scene_corrs;
   }


  std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> >  motionMatrixes(const pcl::PointCloud<pcl::PointXYZ>::Ptr model, const pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints, const pcl::PointCloud<pcl::Normal>::Ptr model_normals, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints,  const pcl::PointCloud<pcl::Normal>::Ptr scene_normals,const pcl::CorrespondencesPtr model_scene_corrs,std::vector<pcl::Correspondences> &clustered_corrs){
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    //std::vector<pcl::Correspondences> clustered_corrs;

    #ifdef Hough_3d
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf (new pcl::PointCloud<pcl::ReferenceFrame> ());
    pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZ, pcl::Normal, pcl::ReferenceFrame> rf_est;
    rf_est.setFindHoles (true);
    float rf_rad_ (0.015f); /// Parameter to tune 
    rf_est.setRadiusSearch (rf_rad_);
    rf_est.setInputCloud (model_keypoints);
    rf_est.setInputNormals (model_normals);
    rf_est.setSearchSurface (model);
    rf_est.compute (*model_rf);

    rf_est.setInputCloud (scene_keypoints);
    rf_est.setInputNormals (scene_normals);
    rf_est.setSearchSurface (scene);
    rf_est.compute (*scene_rf);

    // Clustering 
    pcl::Hough3DGrouping<pcl::PointXYZ, pcl::PointXYZ, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);
    clusterer.setHoughBinSize (cg_size_);
    clusterer.setHoughThreshold (cg_thresh_);
    clusterer.setUseInterpolation (true);
    clusterer.setUseDistanceWeight (false);

    clusterer.setInputCloud (model_keypoints);
    clusterer.setInputRf (model_rf);
    clusterer.setSceneCloud (scene_keypoints);
    clusterer.setSceneRf (scene_rf);
    clusterer.setModelSceneCorrespondences (model_scene_corrs);

    clusterer.recognize (rototranslations, clustered_corrs);

    return rototranslations;

    #endif

    #ifdef Geometric_consistency
    pcl::GeometricConsistencyGrouping<pcl::PointXYZ, pcl::PointXYZ> gc_clusterer;
    float cg_size_ (0.01f);
    float cg_thresh_ (5.0f);
    gc_clusterer.setGCSize (cg_size_);
    gc_clusterer.setGCThreshold (cg_thresh_);

    gc_clusterer.setInputCloud (model_keypoints);
    gc_clusterer.setSceneCloud (scene_keypoints); 
    gc_clusterer.setModelSceneCorrespondences (model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    gc_clusterer.recognize (rototranslations, clustered_corrs);
    return rototranslations;

    #endif


  }
/*  /////OPalanhare Nirgun O nyare I am taking sphere right now    /////////////////////
  float* likelihood(pcl::PointXYZ zCap, pcl::PointCloud<PointXYZ> zFrustum){
    float wHit = 0.25;
    float wShort = 0.25;
    float wMax = 0.25;
    float wRand = 0.25;

    float zMax = 100;
    float Lambda ;
    float eta ;

    float sigma_hit;

    float likelihood_array[2];
    likelihood_array[0]=1;
    likelihood_array[1]=1;

    for(int i=0;i<zFrustum.size();i++){ float prob=0;
      if(zFrustum.points[i]<zMax)prob += wHit*eta*(1/(sqrt(2*3.14)*sigma_hit))*exp(-(zCap-zFrustum.points[i])*(zCap-zFrustum.points[i])/(2*sigma_hit*sigma_hit));
      if(zFrustum.points[i]<zCap)prob+= wShort*eta*Lambda*exp(-1*Lambda*zFrustum.points[i]);
      if(zFrustum.points[i]==zMax) prob+= wMax;
      if(zFrustum.points[i]<zMax) prob += (wRand*1)/zMax
      likelihood_array[0]=likelihood_array[0]*prob;
    }

    for(int i=0;i<zFrustum.size();i++){ float prob=0;
      if(zFrustum.points[i]<zCap)prob+= wShort*eta*Lambda*exp(-1*Lambda*zFrustum.points[i]);
      if(zFrustum.points[i]==zMax) prob+= wMax;
      if(zFrustum.points[i]<zMax) prob += (wRand*1)/zMax
      likelihood_array[1]=likelihood_array[1]*prob;
    }
    return likelihood_array;



  }



  float priorEstimation(pcl::PointXYZ zCap, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints ,  pcl::PointCloud<pcl::PointXYZ> zSurroundingSphere ){
    pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints = intersection(keypoints,zSurroundingSphere);
    float sigma_w;
    float prob=0;
    float normalizer=0;
    for(int i=0;i<inlierPoints->points.size();i++){
      pcl::PointXYZ r;
      r.x=inlierPoints->points.x-zCap.x;
      r.y=inlierPoints->points.y-zCap.y;
      r.z=inlierPoints->points.z-zCap.z;
      prob+= (1/(sigma_hit*sqrt(2*3.14)))*exp(-(distance_origin(r))/(2*sigma_w*sigma_w));
    }

    for(int i=0;i<zSurroundingSphere->points.size();i++){
      pcl::PointXYZ r;
      r.x=inlierPoints->points.x-zCap.x;
      r.y=inlierPoints->points.y-zCap.y;
      r.z=inlierPoints->points.z-zCap.z;
      normalizer+= (1/(sigma_hit*sqrt(2*3.14)))*exp(-(distance_origin(r))/(2*sigma_w*sigma_w));
    }

    return prob/normalizer;



  }*/

   



    


};

int main(int argc, char **argv){
	ros::init(argc, argv, "MotionEstimation");
	MotionEstimation node;
	ros::spin();
	return 0;
}



