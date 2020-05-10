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





	    
	    sensor_msgs::PointCloud2 all_points_msg;
	    pcl::toROSMsg(*g_all_pc, all_points_msg);

	    all_points_msg.header.stamp = laserCloudMsg->header.stamp;
	    all_points_msg.header.frame_id = laserCloudMsg->header.frame_id;
	    all_points_pub.publish(all_points_msg);
	    g_all_pc->clear();


