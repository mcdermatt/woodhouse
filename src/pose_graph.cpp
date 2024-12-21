#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>  // Add this line for tf2_ros::Buffer
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <random>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf2/exceptions.h>
#include <cmath>
#include "woodhouse/KeyframeData.h"
#include "woodhouse/GetTheseClouds.h"

using namespace std;
using namespace Eigen;

// struct Keyframe {
//     int id; // Unique keyframe ID
//     pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud; // Point cloud for the keyframe
//     ros::Time timestamp; // Time of the keyframe
// };

// class KeyframeManager {
// public:
//     void addKeyframe(int id, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& timestamp) {
//         Keyframe kf;
//         kf.id = id;
//         kf.point_cloud = cloud;
//         kf.timestamp = timestamp;
//         keyframes_[id] = kf;
//     }

//     std::shared_ptr<Keyframe> getKeyframe(int id) {
//         if (keyframes_.find(id) != keyframes_.end()) {
//             return std::make_shared<Keyframe>(keyframes_[id]);
//         }
//         return nullptr;
//     }

//     const std::unordered_map<int, Keyframe>& getAllKeyframes() const {
//         return keyframes_;
//     }

//     void removeKeyframe(int id) {
//         keyframes_.erase(id);
//     }

// private:
//     std::unordered_map<int, Keyframe> keyframes_;
// };



class PoseGraphNode {
public:
    PoseGraphNode(ros::NodeHandle& nh){
        // Set up ROS subscribers and publishers
        keyframe_sub_ = nh.subscribe("/keyframe_data", 10, &PoseGraphNode::keyframeDataCallback, this);
        get_these_clouds_sub_ = nh.subscribe("/get_these_clouds", 10, &PoseGraphNode::getTheseCloudsCallback, this);

        ros::Rate rate(10);

    }

    void keyframeDataCallback(const woodhouse::KeyframeData::ConstPtr& msg) {
        // // Print the scan_index from the received message
        // ROS_INFO("Received keyframe with scan_index: %d", msg->scan_index);

        // Optionally, if you want to also print out the point cloud size (for debugging purposes)
        ROS_INFO("Point cloud size: %zu", msg->point_cloud.data.size());
    }

    void getTheseCloudsCallback(const woodhouse::GetTheseClouds::ConstPtr& msg) {
        // Print the scan_index from the received message
        std::cout << "Need to find the clouds for indices:" << msg->scan1_index << " and " << msg->scan2_index << std::endl;
    }


private:
    // KeyframeManager keyframe_manager;

    ros::Subscriber keyframe_sub_;
    ros::Subscriber get_these_clouds_sub_;

    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
    }

    sensor_msgs::PointCloud2 convertEigenToROS(const Eigen::MatrixXf& eigenPointCloud, const std_msgs::Header& header) {
    pcl::PointCloud<pcl::PointXYZ> pclPointCloud;
        // Assuming each row of the Eigen matrix represents a point (x, y, z)
        for (int i = 0; i < eigenPointCloud.rows(); ++i) {
            pcl::PointXYZ point;
            point.x = eigenPointCloud(i, 0);
            point.y = eigenPointCloud(i, 1);
            point.z = eigenPointCloud(i, 2);
            pclPointCloud.push_back(point);
        }

        // Convert PCL point cloud to ROS message
        sensor_msgs::PointCloud2 rosPointCloud;
        pcl::toROSMsg(pclPointCloud, rosPointCloud);
        rosPointCloud.header = header;  // Set the header from the input

        return rosPointCloud;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle nh;
    PoseGraphNode pg_node(nh);
    ros::spin();
    return 0;
}


