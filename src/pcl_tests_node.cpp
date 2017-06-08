#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>  

#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

#include <tf/transform_listener.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PC;

class Reg {
	std::map<std::string, PC::Ptr> map;
	
	bool isPCValid(PC::Ptr pc);

public:
	Reg(){};
	void callback(const PC::Ptr pc){
		std::string frame_id = pc->header.frame_id;
		ROS_INFO("Frame id: %s", frame_id.c_str());
		ROS_INFO("Is dense: %d", (int) pc->is_dense);	
		//if (map.count(frame_id) > 0) return;
		//if (!isPCValid(pc)) return;

		//map[pc->header.frame_id] = pc;	
	};
};


int main (int argc, char** argv){
	ros::init(argc, argv, "pcl_tests_node");
	ros::NodeHandle node("~");

	std::string cloud_topic;
	if (!node.getParam("cloud_topic", cloud_topic)){
		ROS_ERROR("Needs cloud topic parameter -- use launch file to start");
		return 1;
	}

	Reg reg;

	//Testing
	ros::Subscriber sub1 = node.subscribe(cloud_topic, 2, &Reg::callback, &reg);
	
	//ros::Subscriber sub1 = node.subscribe("/first_device/depth_registered/points", 2, &Reg::callback, &reg);	
	//ros::Subscriber sub2 = node.subscribe("/second_device/depth_registered/points", 2, &Reg::callback, &reg);	


	ros::spin();
}
