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


class ScanContextNode {
public:
    ScanContextNode() : nh_("~"), initialized_(false), tfListener_(tfBuffer_) {
        // Set up ROS subscribers and publishers
        pointcloud_sub_ = nh_.subscribe("/velodyne_points", 10, &ScanContextNode::pointcloudCallback, this); //use when connected to Velodyne VLP-16
        keyframe_data_pub_ =  nh_.advertise<woodhouse::KeyframeData>("/keyframe_data", 10);
        get_these_clouds_pub_ =  nh_.advertise<woodhouse::GetTheseClouds>("/get_these_clouds", 10);

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

        // get overall odometry estimates from ICET
        try {
            // Wait for the transform to become available
            ros::Duration timeout(4.0);
            if (!tfBuffer_.canTransform("map", "velodyne", ros::Time(0), timeout)) {
                ROS_ERROR("Transform from 'map' to 'velodyne' not available within the timeout.");
            }

            // Look up the transform
            geometry_msgs::TransformStamped transformStamped = tfBuffer_.lookupTransform("map", "velodyne", ros::Time(0));

            // Extract translation and rotation
            trans_ = transformStamped.transform.translation;
            rot_ = transformStamped.transform.rotation;
            rot_eigen.x() = rot_.x;
            rot_eigen.y() = rot_.y;
            rot_eigen.z() = rot_.z;
            rot_eigen.w() = rot_.w;

            // std::cout << "Translation: [" << trans_.x << ", " << trans_.y << ", " << trans_.z << "]" << std::endl;
            // std::cout << "Rotation: [" << rot_.x << ", " << rot_.y << ", " << rot_.z << ", " << rot_.w << "]" << std::endl;

        } catch (const tf2::TransformException& ex) {
            ROS_ERROR("Transform error: %s", ex.what());
        }
        frames_since_last_kf++;
        // cout<< frames_since_last_kf << " " << pose_at_last_kf << endl;        

        dist_since_last_kf = sqrt(pow(pose_at_last_kf[0] - trans_.x, 2) 
                                + pow(pose_at_last_kf[1] - trans_.y, 2) 
                                + pow(pose_at_last_kf[2] - trans_.z, 2));

        // std::cout << "dist since last kf " << dist_since_last_kf << endl;

        // Conditions to make a new keyframe
        if (dist_since_last_kf > dist_thresh && frames_since_last_kf > frame_thresh){

            // Compute the scan context descriptor
            sc_manager_.makeAndSaveScancontextAndKeys(*pcl_cloud_intensity);
            ROS_INFO("Scan context generated and saved!");

            Eigen::MatrixXd& latest_context = sc_manager_.polarcontexts_.back();
            Eigen::MatrixXd latest_keyring = sc_manager_.makeRingkeyFromScancontext(latest_context);

            // Flatten the latest context
            Eigen::VectorXd latest_context_flat = Eigen::Map<Eigen::VectorXd>(latest_context.data(), latest_context.size());

            int loop_id = -1;
            if (keyframeCount > 0){
                std::pair<int, float> result = sc_manager_.detectLoopClosureID();
                loop_id = result.first;
                float yaw_diff_in_rad = result.second;
                std::cout << "loop_id:" << loop_id << std::endl;
            }

            //create keyframe data msg
            woodhouse::KeyframeData keyframe_msg;
            keyframe_msg.scan_index = keyframeCount;
            keyframe_msg.last_scan_index = keyframeCount - 1;
            keyframe_msg.point_cloud = *msg;
            std::vector<float> flattened_context;        
            for (int i = 0; i < latest_context.rows(); ++i) {
                for (int j = 0; j < latest_context.cols(); ++j) {
                    flattened_context.push_back(static_cast<float>(latest_context(i, j)));
                }
            }
            keyframe_msg.scan_context = flattened_context;
            std::vector<float> flattened_keyring;        
            for (int i = 0; i < latest_keyring.rows(); ++i) {
                for (int j = 0; j < latest_keyring.cols(); ++j) {
                    flattened_keyring.push_back(static_cast<float>(latest_keyring(i, j)));
                }
            }
            //get relative rotation since last keyframe
            rot_eigen.normalize();
            quat_at_last_kf.normalize();
            Eigen::Quaternionf q_relative = rot_eigen * quat_at_last_kf.inverse();

            //TODO -- these xyz positions are relative to WORLD coordinate system 
            // we need to bring them into the frame of the previous scan to use in pose graph optimization...
            keyframe_msg.odom_constraint.position.x = (trans_.x - pose_at_last_kf[0]);
            keyframe_msg.odom_constraint.position.y = (trans_.y - pose_at_last_kf[1]);
            keyframe_msg.odom_constraint.position.z = (trans_.z - pose_at_last_kf[2]);
            keyframe_msg.odom_constraint.orientation.x = q_relative.x();
            keyframe_msg.odom_constraint.orientation.y = q_relative.y();
            keyframe_msg.odom_constraint.orientation.z = q_relative.z();
            keyframe_msg.odom_constraint.orientation.w = q_relative.w();

            keyframe_msg.ring_keys = flattened_keyring;
            keyframe_data_pub_.publish(keyframe_msg);

            //create message to tell pose graph node to grab these scans (only if valid match)
            if (loop_id > -1){
                woodhouse::GetTheseClouds get_clouds_msg;
                get_clouds_msg.scan1_index = keyframeCount;
                get_clouds_msg.scan2_index = loop_id;
                get_these_clouds_pub_.publish(get_clouds_msg);
            }

            pose_at_last_kf[0] = static_cast<float>(trans_.x);
            pose_at_last_kf[1] = static_cast<float>(trans_.y);
            pose_at_last_kf[2] = static_cast<float>(trans_.z);
            quat_at_last_kf = rot_eigen;
            frames_since_last_kf = 0;
            keyframeCount++;

            // //DEBUG save csv
            // std::ofstream file("latest_scan_context.csv");
            // if (file.is_open()) {
            //     file << latest_context.format(Eigen::IOFormat(Eigen::FullPrecision, Eigen::DontAlignCols, ",", "\n"));
            //     file.close();
            // }
            // //DEBUG display scan context as image using opencv
            // cv::Mat context_image(latest_context.rows(), latest_context.cols(), CV_64F, const_cast<double*>(latest_context.data()));
            // cv::normalize(context_image, context_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // cv::Mat resized_image;
            // int scale_factor = 10; // Increase this for larger size
            // cv::resize(context_image, resized_image, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
            // // Display the resized image
            // cv::imshow("Scan Context", resized_image);
            // cv::waitKey(0);
        }

    }

    Eigen::VectorXf X0;

private:
    const float dist_thresh = 1.;
    const int frame_thresh = 10;

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
    Eigen::Quaternionf quat_at_last_kf;
    Eigen::Quaternionf rot_eigen;
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
    ros::init(argc, argv, "scan_context_node");
    ScanContextNode sc_node;
    ros::spin();
    return 0;
}
