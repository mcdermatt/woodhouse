// #include <ros/ros.h>
// #include <tf/transform_broadcaster.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>  // Add this line for tf2_ros::Buffer
// #include <pcl/registration/icp.h>
// #include <Eigen/Dense>
// #include <random>
// #include <nav_msgs/Odometry.h>
// #include <geometry_msgs/Quaternion.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <geometry_msgs/Pose.h>
// #include <std_msgs/String.h>
// #include <tf/tf.h>
// #include <tf2/exceptions.h>
// #include <cmath>
// #include "woodhouse/KeyframeData.h"
// #include "woodhouse/GetTheseClouds.h"
// #include <map>

// using namespace std;
// using namespace Eigen;

// class LoopCloserNode {
// public:
//     LoopCloserNode(ros::NodeHandle& nh){
//         // Set up ROS subscribers and publishers
//         get_these_clouds_pub_ =  nh_.advertise<woodhouse::GetTheseClouds>("/get_these_clouds", 10);

//         ros::Rate rate(10);
//         initializePoseCSV("pose_data.csv");

//     }

//     map<int, Keyframe> frames_;


// private:

//     ros::Subscriber keyframe_sub_;
//     ros::Subscriber get_these_clouds_pub_;

//     Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
//         Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
//         for (size_t i = 0; i < pcl_cloud->size(); ++i) {
//             eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
//         }
//         return eigen_matrix;
//     }

//     sensor_msgs::PointCloud2 convertEigenToROS(const Eigen::MatrixXf& eigenPointCloud, const std_msgs::Header& header) {
//     pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
//         // Assuming each row of the Eigen matrix represents a point (x, y, z)
//         for (int i = 0; i < eigenPointCloud.rows(); ++i) {
//             pcl::PointXYZ point;
//             point.x = eigenPointCloud(i, 0);
//             point.y = eigenPointCloud(i, 1);
//             point.z = eigenPointCloud(i, 2);
//             pclPointCloud.push_back(point);
//         }

//         // Convert PCL point cloud to ROS message
//         sensor_msgs::PointCloud2 rosPointCloud;
//         pcl::toROSMsg(pclPointCloud, rosPointCloud);
//         rosPointCloud.header = header;  // Set the header from the input

//         return rosPointCloud;
//     }

// };

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "loop_closer_node");
//     ros::NodeHandle nh;
//     PoseGraphNode pg_node(nh);
//     ros::spin();
//     return 0;
// }


