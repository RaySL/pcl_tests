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

class CamTFCal {
private:
	// Needs a TF and PC for both cameras
	tf::TransformListener m_tf_listener;
	ros::Publisher m_tf_publisher;

	ros::NodeHandle m_nh;

	ros::Subscriber m_sub;
	
	std::string m_frame_id;
	PC::Ptr m_cloud;

	void callback(const PC::Ptr cloud);

public:
	CamTFCal();

	// Needs a method to begin calibration
	void calibrate();
	// Needs a method to subscribe to the TF and PC
	void subscribe(std::string);

	~CamTFCal();
};

// Define a new point representation for < x, y, z, curvature >
// From Andreas's twopcd6.cpp
class MyPointRepresentation : public pcl::PointRepresentation<pcl::PointNormal>
{
  using pcl::PointRepresentation<pcl::PointNormal>::nr_dimensions_;
public:
  MyPointRepresentation ()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray (const pcl::PointNormal &p, float * out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};
