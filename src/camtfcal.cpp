#include <pcl_tests/camtfcal.h>

CamTFCal::CamTFCal(){

}

void CamTFCal::calibrate(){
	//Try to get a transform from the first cloud to the next
	//tf::StampedTransform transform;

	//Wait up to two seconds for transform, if you don't get it, continue
	//if (m_tf_listener.waitForTransform(m_frame1, m_frame2, ros::Time(0), ros::Duration(2.0)))
	//{
	//	m_tf_listener.lookupTransform(m_frame1, m_frame2, ros::Time(0), transform);
	//} else {
	//	ROS_WARN("No transform from %s to %s arrived in 2 seconds", m_frame1.c_str(), m_frame2.c_str());
	//	return;
	//}
}	

void CamTFCal::subscribe(std::string cloud) {
	m_sub = m_nh.subscribe(cloud, 2, &CamTFCal::callback, this);
}

void CamTFCal::callback(const PC::Ptr cloud){
	pcl::PassThrough < pcl::PointXYZRGB > pass;

	// Restrict z coordinate
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(0.0, 1.4);
	//pass.setInputCloud(cloud);
	//pass.filter(*cloud);
	
	// Restrict x coordinate
	//pass.setFilterFieldName("x");
	//pass.setFilterLimits(-1.3, 1.3);
	//pass.setInputCloud(cloud);
	//pass.filter(*cloud);

	// Voxelize the point cloud, from Andreas's twopcd6.cpp
	PC::Ptr cloud_vox(new PC);
	pcl::VoxelGrid < pcl::PointXYZRGB > vox;
	vox.setInputCloud(cloud);
	vox.setLeafSize(0.005f, 0.005f, 0.005f);
	vox.filter(*cloud_vox);

	//Try to get a transform between the clouds
	tf::StampedTransform transform;

	//Wait up to two seconds for transform, if you don't get it, continue
	if (m_tf_listener.waitForTransform(cloud->header.frame_id, m_frame_id, ros::Time(0), ros::Duration(2.0)))
	{
		m_tf_listener.lookupTransform(cloud->header.frame_id, m_frame_id, ros::Time(0), transform);
	} else {
		ROS_WARN("No transform from %s to %s arrived in 2 seconds", cloud->header.frame_id.c_str(), m_frame_id.c_str());
		return;
	}

	//We didn't fail to get the transform, so use it
	PC::Ptr temp_cloud(new PC);
	//Args are frame to transform to, input, output, and transform listener to use
	//So after this, temp_cloud is the cloud the iterator points to, transformed into the frame of the first cloud
	pcl_ros::transformPointCloud(m_frame_id, *cloud, *temp_cloud, m_tf_listener);
	if (temp_cloud->points.size() == 0)
	{
		ROS_WARN("Empty cloud after transformation.");
		return;
	}

	//Estimate normals for first and new/temp cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr begin_norm(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr temp_norm(new pcl::PointCloud<pcl::PointNormal>);

	pcl::NormalEstimation < pcl::PointXYZRGB, pcl::PointNormal > norm_est;
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	norm_est.setSearchMethod(tree);
	norm_est.setKSearch(30);

	norm_est.setInputCloud(m_cloud);
	norm_est.compute(*begin_norm);
	pcl::copyPointCloud(m_cloud, *begin_norm);

	norm_est.setInputCloud(temp_cloud);
	norm_est.compute(*temp_norm);
	pcl::copyPointCloud(*temp_cloud, *temp_norm);

	//Compute curvatures
	MyPointRepresentation point_representation;
	//Weight the 'curvature' dimension so that it is balanced against x, y, and z
	float alpha[4] =
	{ 1.0, 1.0, 1.0, 1.0 };
	point_representation.setRescaleValues(alpha);

	//Set up for Iterative Closest Point (ICP)
	pcl::IterativeClosestPointNonLinear < pcl::PointNormal, pcl::PointNormal > reg;
	reg.setTransformationEpsilon(1e-6);
	//Set the maximum distance between two correspondences (src<->tgt)
	//Note: adjust this based on the size of your datasets
	//Andreas used 0.01, 0.013, 0.02, and 0.04, judging by his comments.
	//Graphing the registration fitness over 0.01-0.35 showed abrupt drops at ~0.03 and again at ~0.12
	reg.setMaxCorrespondenceDistance(0.04); //pretty decent
	//reg.setMaxCorrespondenceDistance(0.07); //OK
	//reg.setMaxCorrespondenceDistance(0.01); //bad
	//reg.setMaxCorrespondenceDistance(0.125); //Good, but not good enough

	//Set the point representation
	reg.setPointRepresentation(boost::make_shared<
			const MyPointRepresentation>(point_representation));
	//Inputs are the normal point clouds
	reg.setInputSource(temp_norm);
	reg.setInputTarget(begin_norm);
	temp_norm->header.frame_id = m_frame_id;
	//Publish the untransformed image
	//debug_pub.publish(temp_norm);

	begin_norm->header.frame_id = m_frame_id;
	//debug_pub.publish(begin_norm);

	//Align into the first point cloud
	pcl::PointCloud<pcl::PointNormal>::Ptr reg_result = temp_norm;
	reg.align(*reg_result);

	//Calculate the final transformation
	Eigen::Matrix4f icpTransf = reg.getFinalTransformation();
	//Transform it
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr icp_final_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	//pcl::transformPointCloud(*temp_cloud, *icp_final_cloud, icpFinal);
	pcl::transformPointCloud(*temp_cloud, *icp_final_cloud, icpTransf);

	//Combine the point clouds
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_combined = *icp_final_cloud + *(m_cloud);

	pub.publish(cloud_combined);

	// FIGURE OUT WHAT TO PUBLISH ----
	
	// remove NANs
//	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_combined_nan(new pcl::PointCloud<
//			pcl::PointXYZRGB>);
//	std::vector<int> indices;
//	pcl::removeNaNFromPointCloud(*cloud_combined, *cloud_combined_nan, indices); //TODO, can I do this inline?

}

















