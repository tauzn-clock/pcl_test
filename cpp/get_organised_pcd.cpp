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

#include "data_conversion.cpp"

ros::Publisher cloud_pub, marker_array_pub;
sensor_msgs::CameraInfo camera_info;
visualization_msgs::MarkerArray marker_array;

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
    pcl::PointCloud<pcl::PointXYZ> cloud = DepthMsgToPointCloud(msg, camera_info);

    ROS_INFO("Point Cloud Size: %zu", cloud.size());

    // Normal Estimate
    pcl::PointCloud<pcl::Normal> normal;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne_normals;
    ne_normals.setNormalEstimationMethod(ne_normals.COVARIANCE_MATRIX);
    ne_normals.setMaxDepthChangeFactor(0.1f);
    ne_normals.setNormalSmoothingSize(20.0f);
    ne_normals.setInputCloud(cloud.makeShared());
    ne_normals.compute(normal);

    // Save the point cloud as a PLY file
    // pcl::io::savePCDFileASCII("/scratchdata/organised_pcd.pcd", cloud);

    // Find planes using organised multiplane segmentation
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> seg;
    seg.setInputCloud(cloud.makeShared());
    seg.setInputNormals(normal.makeShared());

    // Call segment function to obtain set of plane models in the input cloud
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::ModelCoefficients> models;
    seg.segment(models, cluster_indices);
    
    // Publish the point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);

    /* Very computationally intensive to run, only do it for testing
    marker_array = PointCloudWithNormalsToMarkerArray(cloud, normal);
    marker_array_pub.publish(marker_array);
    */
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "get_organised_pcd");

    // Create a node handle
    ros::NodeHandle nh;

    ros::Subscriber camera_info_sub = nh.subscribe("/camera/depth/camera_info", 1, cameraInfoCallback);
    ros::Subscriber depth_image_sub = nh.subscribe("/camera/depth/image_raw", 10, depthImageCallback);

    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/camera/organised_pcd", 1);
    marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("/camera/organised_pcd_normals", 1);

    // Spin to keep the node alive
    ros::spin();

    return 0;
}
