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
#include "hsv.cpp"
#include <array>
#include <yaml-cpp/yaml.h>

ros::Publisher cloud_pub, marker_array_pub;
sensor_msgs::CameraInfo camera_info;
visualization_msgs::MarkerArray marker_array;
YAML::Node config;

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

    // Normal Estimate
    pcl::PointCloud<pcl::Normal> normal;
    pcl::IntegralImageNormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne_normals;
    ne_normals.setNormalEstimationMethod(ne_normals.COVARIANCE_MATRIX);
    ne_normals.setMaxDepthChangeFactor(config["normal_max_depth_change_factor"].as<float>());
    ne_normals.setNormalSmoothingSize(config["normal_normal_smoothing_size"].as<float>());
    ne_normals.setInputCloud(cloud.makeShared());
    ne_normals.compute(normal);

    // Save the point cloud as a PLY file
    // pcl::io::savePCDFileASCII("/scratchdata/organised_pcd.pcd", cloud);

    // Find planes using organised multiplane segmentation
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZRGB, pcl::Normal, pcl::Label> seg;
    seg.setInputCloud(cloud.makeShared());
    seg.setInputNormals(normal.makeShared());
    seg.setMinInliers(config["segment_min_inliers"].as<int>());
    seg.setAngularThreshold(config["segment_angular_threshold"].as<float>());
    seg.setDistanceThreshold(config["segment_distance_threshold"].as<float>());
    seg.setMaximumCurvature(config["segment_maximum_curvature"].as<float>());


    // Call segment function to obtain set of plane models in the input cloud
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<pcl::ModelCoefficients> models;
    pcl::PointCloud<pcl::Label> labels;
    std::vector<pcl::PointIndices> label_indices;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > centroids;
    std::vector <Eigen::Matrix3f, Eigen::aligned_allocator<Eigen::Matrix3f> > covariances;
    seg.segment(models, cluster_indices,centroids, covariances, labels, label_indices);
    
    // Print out the number of planes found
    ROS_INFO("Number of planes found: %zu", models.size());
    for(auto model : models)
    {
        ROS_INFO("Model Coefficients: %f %f %f %f", model.values[0], model.values[1], model.values[2], model.values[3]);
    }

    // Print out the number of indexes in each cluster
    for(int i=0; i<cluster_indices.size(); i++)
    {
        ROS_INFO("Cluster %d Size: %zu", i, cluster_indices[i].indices.size());
        std::array<float, 3> color = GetHSVColor(i, cluster_indices.size());
        for (auto index : cluster_indices[i].indices)
        {
            cloud[index].r = (int)(color[0] * 255);
            cloud[index].g = (int)(color[1] * 255);
            cloud[index].b = (int)(color[2] * 255);
        }
    }

    // Publish the point cloud
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();
    cloud_pub.publish(cloud_msg);

    /* Very computationally intensive to run, only do it for testing */
    if (config["normal_visualise"].as<bool>())
    {
        visualization_msgs::MarkerArray marker_array = PointCloudWithNormalsToMarkerArray(cloud, normal);
        marker_array_pub.publish(marker_array);
    }
}

int main(int argc, char** argv)
{
    std::cout << argv[1] << std::endl;

    config = YAML::LoadFile("/catkin_ws/src/pcl_test/cpp/organised.yaml");

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
