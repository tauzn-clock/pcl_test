#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int W;
int H;
float fx;
float fy;
float cx;
float cy;

ros::Publisher cloud_pub;

void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
    W = msg->width;
    H = msg->height;
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
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

    ROS_INFO("Camera Parameters: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);

    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = W;
    cloud.height = H;
    cloud.is_dense = false;
    cloud.points.resize (W*H);
    
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> ::iterator pt_iter = cloud.begin ();
    for (int v = 0; v < (int)cloud.height; ++v)
    {
        int depth_idx = 0;
        for (int u = 0; u < (int)cloud.width; ++u, ++depth_idx, ++pt_iter)
        {
            pcl::PointXYZ& pt = *pt_iter;
            float Z = msg->data[depth_idx];
            ROS_INFO("Z: %.2f", Z);

            // Check for invalid measurements
            if (std::isnan (Z))
            {
                pt.x = pt.y = pt.z = Z;
            }
            else // Fill in XYZ
            {
                pt.x = ((float)u - cx) * Z * fx;
                pt.y = ((float)v - cy) * Z * fy;
                pt.z = Z;
            }
        }
    }

    ROS_INFO("Point Cloud Size: %zu", cloud.size());

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;

    // Publish the data
    cloud_pub.publish(output);
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

    // Spin to keep the node alive
    ros::spin();

    return 0;
}
