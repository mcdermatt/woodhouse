#include "scancontext/Scancontext.h"
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

struct Keyframe {
    int id; // Unique keyframe ID
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud; // Point cloud for the keyframe
    ros::Time timestamp; // Time of the keyframe
};

class KeyframeManager {
public:
    void addKeyframe(int id, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const ros::Time& timestamp) {
        Keyframe kf;
        kf.id = id;
        kf.point_cloud = cloud;
        kf.timestamp = timestamp;
        keyframes_[id] = kf;
    }

    std::shared_ptr<Keyframe> getKeyframe(int id) {
        if (keyframes_.find(id) != keyframes_.end()) {
            return std::make_shared<Keyframe>(keyframes_[id]);
        }
        return nullptr;
    }

    const std::unordered_map<int, Keyframe>& getAllKeyframes() const {
        return keyframes_;
    }

    void removeKeyframe(int id) {
        keyframes_.erase(id);
    }

private:
    std::unordered_map<int, Keyframe> keyframes_;
};



class PoseGraphNode {
public:
    PoseGraphNode() : nh_("~"), initialized_(false), tfListener_(tfBuffer_) {
        // Set up ROS subscribers and publishers
        keyframe_sub_ = nh.subscribe("/keyframe_data", 10, &PoseGraphNode::keyframeDataCallback, this);
        // keyframe_sub_ = nh.subscribe("/keyframe_data", 10, &PoseGraphNode::keyframeDataCallback, this);
        // keyframe_data_pub_ =  nh_.advertise<woodhouse::KeyframeData>("/keyframe_data", 10);
        // get_these_clouds_pub_ =  nh_.advertise<woodhouse::GetTheseClouds>("/get_these_clouds", 10);

        ros::Rate rate(10);
        pose_at_last_kf.resize(3);
        pose_at_last_kf << 0., 0., 0.;
        rot_.w = 1.0; //needed for quat initialization
    }

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        auto before = std::chrono::system_clock::now();
        auto beforeMs = std::chrono::time_point_cast<std::chrono::milliseconds>(before);
        frameCount++;

        // Convert PointCloud2 to PCL PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *pcl_cloud);
        // Convert PCL PointCloud to Eigen::MatrixXf
        // Eigen::MatrixXf pcl_matrix = convertPCLtoEigen(pcl_cloud);

        // Convert pcl::PointXYZ to pcl::PointXYZI
        pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_intensity(new pcl::PointCloud<pcl::PointXYZI>);
        pcl_cloud_intensity->reserve(pcl_cloud->size());
        for (const auto& point : pcl_cloud->points) {
            pcl::PointXYZI point_intensity;
            point_intensity.x = point.x;
            point_intensity.y = point.y;
            point_intensity.z = point.z;
            point_intensity.intensity = 0.0f; // Assign default intensity
            pcl_cloud_intensity->points.push_back(point_intensity);
        }

    }

private:
    const float dist_thresh = 1.;
    const int frame_thresh = 10;

    KeyframeManager keyframe_manager;

    ros::NodeHandle nh_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher keyframe_data_pub_;
    ros::Publisher get_these_clouds_pub_;

    SCManager sc_manager_; // ScanContext manager instance

    pcl::PointCloud<pcl::PointXYZ>::Ptr prev_pcl_cloud_;
    bool initialized_;
    int frameCount = 0;
    int keyframeCount = 0;
    int frames_since_last_kf = 0;
    float dist_since_last_kf = 0.;
    Eigen::VectorXf pose_at_last_kf;
    Eigen::MatrixXf prev_pcl_matrix;

    geometry_msgs::Vector3 trans_ = {};   // for translation (x, y, z)
    geometry_msgs::Quaternion rot_ = {};  // for rotation (x, y, z, w)

    //init variable to hold cumulative homogenous transform
    Eigen::Matrix4f X_homo = Eigen::Matrix4f::Identity(); 

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
    PoseGraphNode pg_node;
    ros::spin();
    return 0;
}


