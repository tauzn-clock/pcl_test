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

pcl::PointCloud<pcl::PointXYZ> DepthMsgToPointCloud(const sensor_msgs::Image::ConstPtr& msg, sensor_msgs::CameraInfo camera_info){

    float fx = camera_info.K[0];
    float fy = camera_info.K[4];
    float cx = camera_info.K[2];
    float cy = camera_info.K[5];
    
    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = msg->width;
    cloud.height = msg->height;
    cloud.is_dense = true;
    cloud.points.resize (msg->width*msg->height);

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

    return cloud;
}