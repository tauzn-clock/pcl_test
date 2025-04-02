#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/integral_image_normal.h>

#include "ahc/AHCPlaneFitter.hpp"
#include "opencv2/opencv.hpp"

#include "data_conversion.cpp"
#include "hsv.cpp"
#include <array>
#include <math.h>
#include <yaml-cpp/yaml.h>

ros::Publisher cloud_pub, marker_array_pub;
sensor_msgs::CameraInfo camera_info;
YAML::Node config;

// pcl::PointCloud interface for ahc::PlaneFitter
template <class PointT>
struct OrganizedImage3D
{
	const pcl::PointCloud<PointT> &cloud;
	// note: ahc::PlaneFitter assumes mm as unit
	const double unitScaleFactor;

	OrganizedImage3D(const pcl::PointCloud<PointT> &c) : cloud(c), unitScaleFactor(1) {}
	OrganizedImage3D(const OrganizedImage3D &other) : cloud(other.cloud), unitScaleFactor(other.unitScaleFactor) {}

	inline int width() const { return cloud.width; }
	inline int height() const { return cloud.height; }
	inline bool get(const int row, const int col, double &x, double &y, double &z) const
	{
		const PointT &pt = cloud.at(col, row);
		x = pt.x * unitScaleFactor;
		y = pt.y * unitScaleFactor;
		z = pt.z * unitScaleFactor;
		return std::isnan(z) == 0; // return false if current depth is NaN
	}
};

namespace global_para
{
	int stdTol_merge = 8, stdTol_init = 5, z_near = 500, z_far = 4000, angleDegree_near = 15, angleDegree_far = 90, similarityDegreeTh_merge = 60, similarityDegreeTh_refine = 30, initType = 0, minSupport = 1000, windowWidth = 10, windowHeight = 10, doRefine = 1;
	double angleRadian_tol = 0.08, unitScaleFactor = 1000, depthSigma = 1.6e-6, depthAlpha = 0.04, depthChangeTol = 0.02, map_to_point_cloud_tf_list[7];
	float min_pt_list[3], max_pt_list[3];
	std::string rviz = "1", debug = "0", map_to_point_cloud_tf = "0 0 0.18 -0.9238795 0 0 0.3826835", min_pt = "-1.5f -1.5f -0.01f", max_pt = "1.5f 1.5f 1.5f";
}

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    camera_info = *msg;
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Print out depth image info
    ROS_INFO("Received Depth Image:");
    ROS_INFO("Width: %d, Height: %d", msg->width, msg->height);
    ROS_INFO("Encoding: %s", msg->encoding.c_str());
    ROS_INFO("Is Bigendian? %s", msg->is_bigendian ? "True" : "False");
    ROS_INFO("Step: %d", msg->step);
    ROS_INFO("Data Size: %zu", msg->data.size());

    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud = DepthMsgToPointCloud(msg, camera_info);

    ROS_INFO("Point Cloud Size: %zu", cloud.size());

    /////////////////////////////// Initializing Plane Segmentation Parameters ////////////////////////////////////
    ahc::PlaneFitter<OrganizedImage3D<pcl::PointXYZRGB>> pf;
    pf.minSupport = global_para::minSupport;
	pf.windowWidth = global_para::windowWidth;
	pf.windowHeight = global_para::windowHeight;
	pf.doRefine = (global_para::doRefine != 0);
	pf.params.initType = (ahc::InitType)global_para::initType;
	// T_mse
	pf.params.stdTol_merge = global_para::stdTol_merge;
	pf.params.stdTol_init = global_para::stdTol_init;
	pf.params.depthSigma = global_para::depthSigma;
	// T_dz
	pf.params.depthAlpha = global_para::depthAlpha;
	pf.params.depthChangeTol = global_para::depthChangeTol;
	// T_ang
	pf.params.z_near = global_para::z_near;
	pf.params.z_far = global_para::z_far;
	pf.params.angle_near = MACRO_DEG2RAD(global_para::angleDegree_near);
	pf.params.angle_far = MACRO_DEG2RAD(global_para::angleDegree_far);
	pf.params.similarityTh_merge = std::cos(MACRO_DEG2RAD(global_para::similarityDegreeTh_merge));
	pf.params.similarityTh_refine = std::cos(MACRO_DEG2RAD(global_para::similarityDegreeTh_refine));

    /////////////////////////////// Segmentation of Planes ////////////////////////////////////

	cv::Mat seg(cloud.height, cloud.width, CV_8UC3);
	//cloud.header.frame_id = "map";
	OrganizedImage3D<pcl::PointXYZRGB> Ixyz(cloud);
	std::vector<std::vector<int>> plane_vertices;
	pf.run(&Ixyz, &plane_vertices, &seg); // Perform plane segmentation

	pcl::PointCloud<pcl::PointXYZRGB> xyzrgb(cloud.width, cloud.height);

	for (int r = 0; r < (int)xyzrgb.height; ++r)
	{
		for (int c = 0; c < (int)xyzrgb.width; ++c)
		{
			pcl::PointXYZRGB &pix = xyzrgb.at(c, r);
			const pcl::PointXYZRGB &pxyz = cloud.at(c, r);
			const cv::Vec3b &prgb = seg.at<cv::Vec3b>(r, c);
			pix.x = pxyz.x;
			pix.y = pxyz.y;
			pix.z = pxyz.z;
			pix.r = prgb(2);
			pix.g = prgb(1);
			pix.b = prgb(0);
		}
	}

    // Publish the point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(xyzrgb, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);
}

int main(int argc, char** argv)
{
    std::cout << argv[1] << std::endl;

    config = YAML::LoadFile("/catkin_ws/src/pcl_test/cpp/ahc.yaml");

    // Initialize ROS
    ros::init(argc, argv, "get_ahc_pcd");

    // Create a node handle
    ros::NodeHandle nh;

    ros::Subscriber camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_raw", 10, depthImageCallback);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/ahc_pcd", 1);

    // Spin to keep the node alive
    ros::spin();

    return 0;
}
