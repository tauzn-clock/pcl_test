#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/io/pcd_io.h>

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
    cloud.is_dense = true;
    cloud.points.resize (W*H);
    
    // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZ> ::iterator pt_iter = cloud.begin ();
    int depth_idx = 0;

    for (int v = 0; v < (int)cloud.height; ++v)
    {
        for (int u = 0; u < (int)cloud.width; ++u, ++pt_iter)
        {
            pcl::PointXYZ& pt = *pt_iter;

            int Z_int;

            if (msg->is_bigendian)
            {
                Z_int = (msg->data[depth_idx] << 8) + msg->data[depth_idx+1];
            }
            else
            {
                Z_int = msg->data[depth_idx] + (msg->data[depth_idx+1] << 8);
            }

            float Z = 0.001 * Z_int;
            depth_idx += 2; // Skip the second byte of the 16-bit depth value

            pt.x = ((float)u - cx) * Z * fx / 100000.0;
            pt.y = ((float)v - cy) * Z * fy / 100000.0;
            pt.z = Z;
        }
    }

    ROS_INFO("Point Cloud Size: %zu", cloud.size());

    // Save the point cloud as a PLY file
    pcl::io::savePCDFileASCII("/scratchdata/organised_pcd.pcd", cloud);

    // Find planes using organised multiplane segmentation
    //pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> seg;
    //seg.setInputCloud(cloud.makeShared());
    //seg.segment();

    // Publish the point cloud
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
