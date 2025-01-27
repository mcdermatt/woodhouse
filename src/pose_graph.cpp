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

class PoseGraphNode {
public:
    PoseGraphNode(ros::NodeHandle& nh){
        // Set up ROS subscribers and publishers
        keyframe_sub_ = nh.subscribe("/keyframe_data", 10, &PoseGraphNode::keyframeDataCallback, this);
        get_these_clouds_sub_ = nh.subscribe("/get_these_clouds", 10, &PoseGraphNode::getTheseCloudsCallback, this);

        ros::Rate rate(10);
        initializePoseCSV("pose_data.csv");

    }

    void keyframeDataCallback(const woodhouse::KeyframeData::ConstPtr& msg) {
        // // Print the scan_index from the received message
        ROS_INFO("Received keyframe with scan_index: %d", msg->scan_index);
        cout << msg->odom_constraint << endl;

        //For debug with Jupyter Notebook:
        // save point cloud to .csv file, titled with keyframe index
        string fn = "keyframe_" + to_string(msg->scan_index) + ".csv";
        savePointCloudToCSV(msg->point_cloud, fn);

        // save odom constraints to text file
        appendPoseToCSV("pose_data.csv", msg->scan_index, msg->odom_constraint);

    }

    void getTheseCloudsCallback(const woodhouse::GetTheseClouds::ConstPtr& msg) {
        // Print the scan_index from the received message
        std::cout << "Need to find the clouds for indices:" << msg->scan1_index << " and " << msg->scan2_index << std::endl;
    }


    void savePointCloudToCSV(const sensor_msgs::PointCloud2& cloud_msg, const std::string& filename) {
        // Convert sensor_msgs/PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(cloud_msg, cloud);

        // Open the output CSV file
        std::ofstream csv_file(filename);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to open file: %s", filename.c_str());
            return;
        }

        for (const auto& point : cloud.points) {
            csv_file << point.x << "," << point.y << "," << point.z << "\n";
        }

        csv_file.close();
        ROS_INFO("PointCloud saved to: %s", filename.c_str());
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

    void initializePoseCSV(const std::string& filename) {
        // Open the CSV file in write mode and add a header
        std::ofstream csv_file(filename, std::ios::out);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to initialize pose CSV file: %s", filename.c_str());
            return;
        }

        csv_file << "scan_index,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w\n";
        csv_file.close();
        ROS_INFO("Initialized pose CSV file: %s", filename.c_str());
    }

    void appendPoseToCSV(const std::string& filename, int scan_index, const geometry_msgs::Pose& pose) {
        // Open the CSV file in append mode
        std::ofstream csv_file(filename, std::ios::app);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to open pose CSV file for appending: %s", filename.c_str());
            return;
        }

        // Append the scan index and pose information
        csv_file << scan_index << ","
                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
                 << pose.orientation.x << "," << pose.orientation.y << "," 
                 << pose.orientation.z << "," << pose.orientation.w << "\n";

        csv_file.close();
        ROS_INFO("Appended pose to CSV: scan_index = %d", scan_index);
    }




};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle nh;
    PoseGraphNode pg_node(nh);
    ros::spin();
    return 0;
}


