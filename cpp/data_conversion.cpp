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

#include <Eigen/Dense>

Eigen::Quaternionf NormalToQuaternion(const Eigen::Vector3f& normal)
{
    Eigen::Vector3f x_axis(1, 0, 0); 
    Eigen::Vector3f axis = x_axis.cross(normal);
    float angle = acos(x_axis.dot(normal));

    // Starting angle from x-axis to normal
    Eigen::Quaternionf quaternion(Eigen::AngleAxisf(angle, axis));
    quaternion.normalize();
    return quaternion;
}

pcl::PointCloud<pcl::PointXYZRGB> DepthMsgToPointCloud(const sensor_msgs::Image::ConstPtr& msg, sensor_msgs::CameraInfo camera_info){

    float fx = camera_info.K[0];
    float fy = camera_info.K[4];
    float cx = camera_info.K[2];
    float cy = camera_info.K[5];

    // Create a point cloud
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    cloud.width = msg->width;
    cloud.height = msg->height;
    cloud.is_dense = true;
    cloud.points.resize (msg->width*msg->height);

     // Fill in the cloud data
    pcl::PointCloud<pcl::PointXYZRGB> ::iterator pt_iter = cloud.begin ();
    int depth_idx = 0;

    for (int v = 0; v < (int)cloud.height; ++v)
    {
        for (int u = 0; u < (int)cloud.width; ++u, ++pt_iter)
        {
            pcl::PointXYZRGB& pt = *pt_iter;

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

            pt.x = ((float)u - cx) * Z / fx;
            pt.y = ((float)v - cy) * Z / fy;
            pt.z = Z;
        }
    }

    return cloud;
}

visualization_msgs::MarkerArray PointCloudWithNormalsToMarkerArray(const pcl::PointCloud<pcl::PointXYZRGB> cloud, const pcl::PointCloud<pcl::Normal> normal)
{
    assert(cloud.size() == normal.size());

    visualization_msgs::MarkerArray marker_array;

    // Iterate through each point in the point cloud
    for (size_t i = 0; i < cloud.size(); i+=100)
    {
        pcl::PointXYZRGB point = cloud.at(i);
        pcl::Normal norm = normal.at(i);

        if (isnan(norm.normal_x) || isnan(norm.normal_y) || isnan(norm.normal_z))
        {
            continue;
        }

        if (point.z < 0.0001)
        {
            continue;
        }

        Eigen::Vector3f normal_vector(norm.normal_x, norm.normal_y, norm.normal_z);
        //Eigen::Vector3f normal_vector(1, 0, 0);
        normal_vector.normalize();
        Eigen::Quaternionf quaternion = NormalToQuaternion(normal_vector);

        //std::cout<<quaternion.x()<<" "<<quaternion.y()<<" "<<quaternion.z()<<" "<<quaternion.w()<<std::endl;

        // Create a marker for the normal (arrow)
        visualization_msgs::Marker normal_marker;
        normal_marker.header.frame_id = "map";  // Set the frame_id for visualization (use appropriate frame)
        normal_marker.header.stamp = ros::Time::now();
        normal_marker.ns = "normals";
        normal_marker.id = i;  // Unique ID for each normal
        normal_marker.type = visualization_msgs::Marker::ARROW;
        normal_marker.action = visualization_msgs::Marker::ADD;
        normal_marker.pose.position.x = point.x;
        normal_marker.pose.position.y = point.y;
        normal_marker.pose.position.z = point.z;
        normal_marker.pose.orientation.x = quaternion.x();
        normal_marker.pose.orientation.y = quaternion.y();
        normal_marker.pose.orientation.z = quaternion.z();
        normal_marker.pose.orientation.w = quaternion.w();
        normal_marker.scale.x = 0.05;  // Arrow shaft radius
        normal_marker.scale.y = 0.02;  // Arrow head radius
        normal_marker.scale.z = 0.0;  // No scaling for Z
        normal_marker.color.r = 1.0f;  // Color of the normal vector (arrow)
        normal_marker.color.g = 0.0f;
        normal_marker.color.b = 0.0f;
        normal_marker.color.a = 1.0;

        // Add normal marker to MarkerArray
        marker_array.markers.push_back(normal_marker);
    }

    return marker_array;
}
