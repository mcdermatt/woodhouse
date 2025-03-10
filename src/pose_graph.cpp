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
#include "woodhouse/HereAreTheClouds.h"
#include "woodhouse/LoopClosed.h"
#include "woodhouse/KeyframePose.h"
#include "woodhouse/KeyframePoses.h"
#include <map>

using namespace std;
using namespace Eigen;

struct Keyframe {
    int scan_index;                          // Index of the scan
    int last_scan_index;                     // Last keyframe index for relative odometry constraint
    sensor_msgs::PointCloud2 point_cloud;    // Point cloud of the keyframe
    std::vector<float> scan_context;         // Scan context feature vector
    std::vector<float> ring_keys;            // Ring key feature vector
    geometry_msgs::Pose odom_constraint;     // Odometry estimate (dead reckoning from registering every raw point cloud )
    geometry_msgs::Pose sequential_keyframe_registration; //directly using ICP to align subsequent keyframes (less drift than odom)
    geometry_msgs::Pose world_pose;          // Absolute pose
};

class PoseGraphNode {
public:
    PoseGraphNode(ros::NodeHandle& nh){
        // Set up ROS subscribers and publishers
        keyframe_sub_ = nh.subscribe("/keyframe_data", 10, &PoseGraphNode::keyframeDataCallback, this);
        get_these_clouds_sub_ = nh.subscribe("/get_these_clouds", 10, &PoseGraphNode::getTheseCloudsCallback, this);
        loop_closure_sub_ = nh.subscribe("/loop_closure_constraint", 10, &PoseGraphNode::loopClosureCallback, this);
        here_are_the_clouds_pub_ =  nh.advertise<woodhouse::HereAreTheClouds>("/here_are_the_clouds", 10);
        keyframe_pose_pub_ = nh.advertise<woodhouse::KeyframePoses>("/optimized_keyframe_poses", 10);

        ros::Rate rate(50);
        initializePoseCSV("pose_data.csv");

        //initialize absolute and relative poses of sensor at first frame to origin
        frames_[0].world_pose.position.x = 0.0;
        frames_[0].world_pose.position.y = 0.0;
        frames_[0].world_pose.position.z = 0.0;
        frames_[0].world_pose.orientation.w = 1.0;
        frames_[0].world_pose.orientation.x = 0.0;
        frames_[0].world_pose.orientation.y = 0.0;
        frames_[0].world_pose.orientation.z = 0.0;
        
        frames_[0].sequential_keyframe_registration.position.x = 0.0;
        frames_[0].sequential_keyframe_registration.position.y = 0.0;
        frames_[0].sequential_keyframe_registration.position.z = 0.0;
        frames_[0].sequential_keyframe_registration.orientation.w = 1.0;
        frames_[0].sequential_keyframe_registration.orientation.x = 0.0;
        frames_[0].sequential_keyframe_registration.orientation.y = 0.0;
        frames_[0].sequential_keyframe_registration.orientation.z = 0.0;

        frames_[0].odom_constraint.position.x = 0.0;
        frames_[0].odom_constraint.position.y = 0.0;
        frames_[0].odom_constraint.position.z = 0.0;
        frames_[0].odom_constraint.orientation.w = 1.0;
        frames_[0].odom_constraint.orientation.x = 0.0;
        frames_[0].odom_constraint.orientation.y = 0.0;
        frames_[0].odom_constraint.orientation.z = 0.0;
    }

    map<int, Keyframe> frames_;

    void keyframeDataCallback(const woodhouse::KeyframeData::ConstPtr& msg) {
        // // Print the scan_index from the received message
        ROS_INFO("Received keyframe with scan_index: %d", msg->scan_index);
        // cout << msg->odom_constraint << endl;

        //add new frame to internal map
        if (frames_.find(msg->scan_index) == frames_.end()){
            Keyframe frame_i;
            frame_i.last_scan_index=msg -> last_scan_index;
            frame_i.point_cloud = msg->point_cloud;
            frame_i.odom_constraint = msg->odom_constraint;
            frames_[msg->scan_index] = frame_i;
            // cout << "holding on to frame" << msg->scan_index << "in pose graph node" << endl;
            updateAbsolutePose(msg->scan_index);
        }        

        //TODO -- insert pose graph optimization here
        //updateOptimizedPoses()

        publishKeyframePoses();

        //For debug with Jupyter Notebook: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // save point cloud to .csv file, titled with keyframe index
        string fn = "keyframe_" + to_string(msg->scan_index) + ".csv";
        savePointCloudToCSV(msg->point_cloud, fn);

        // // save odom constraints to text file
        appendPoseToCSV("pose_data.csv", 0, msg->last_scan_index, msg->scan_index, msg->odom_constraint);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    }

    void loopClosureCallback(const woodhouse::LoopClosed::ConstPtr& msg) {
        cout << "Received loop closure constraint between: " << msg->scan1_index << " and " << msg->scan2_index << endl;
        // cout << msg->loop_closure_constraint << endl;

        // save loop closure constraint to text file ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // subsequent keyframes -- flag with 1
        if (msg->scan2_index - msg->scan1_index == 1){ 
            appendPoseToCSV("pose_data.csv", 1, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            //hold on to SKR and associate with frame
            frames_[msg->scan1_index].sequential_keyframe_registration = msg->loop_closure_constraint;
        }
        //actual loop closure -- flag with 2
        else{
            //ignore instances where ICP diverges
            if (msg->failed_to_converge == false){
                appendPoseToCSV("pose_data.csv", 2, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            }
            //use all
            // appendPoseToCSV("pose_data.csv", 2, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
        }
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    }

    void getTheseCloudsCallback(const woodhouse::GetTheseClouds::ConstPtr& msg) {
        // Print the scan_index from the received message
        std::cout << "Pose_graph_node: publishing clouds for indices:" << msg->scan1_index << " and " << msg->scan2_index << std::endl;

        woodhouse::HereAreTheClouds here_are_the_clouds_msg;
        here_are_the_clouds_msg.scan1_index = msg->scan1_index;
        here_are_the_clouds_msg.scan2_index = msg->scan2_index;
        here_are_the_clouds_msg.point_cloud1 = frames_[msg->scan1_index].point_cloud;
        here_are_the_clouds_msg.point_cloud2 = frames_[msg->scan2_index].point_cloud;

        //provide initial transform from odometry constraint if subsequent point clouds
        if (msg->scan2_index - msg->scan1_index == 1){

            //convert odom constraint quats to roll pitch yaw
            Eigen::Quaterniond q(frames_[msg->scan2_index].odom_constraint.orientation.w, 
                                 frames_[msg->scan2_index].odom_constraint.orientation.x, 
                                 frames_[msg->scan2_index].odom_constraint.orientation.y, 
                                 frames_[msg->scan2_index].odom_constraint.orientation.z);

            // //was creating errors with gimbal lock!!!
            // Eigen::Vector3d rpy = q.toRotationMatrix().eulerAngles(2, 1, 0); // Yaw (Z), Pitch (Y), Roll (X)
            //restry with rotation matrix
            Eigen::Matrix3d R = q.toRotationMatrix();
            double roll  = atan2(R(2,1), R(2,2));
            double pitch = asin(-R(2,0));
            double yaw   = atan2(R(1,0), R(0,0));
            Eigen::Vector3d rpy(roll, pitch, yaw);

            cout << "seeding roll: " << rpy[0] << " pitch: " << rpy[1] << " yaw: " << rpy[2] << endl;

            here_are_the_clouds_msg.X0 = vector<float>{frames_[msg->scan2_index].odom_constraint.position.x,
                                                       frames_[msg->scan2_index].odom_constraint.position.y,
                                                       frames_[msg->scan2_index].odom_constraint.position.z,
                                                       rpy[0], rpy[1], rpy[2]};
        }else{
            //otherwise we are doing a loop closure constraint (use yaw difference provided by scan context 
            //  to seed ICP)
            //TODO-- sign on yaw_seed might need to be flipped...
            float yaw_seed = msg->yaw_diff_rad;
            here_are_the_clouds_msg.X0 = vector<float>{0., 0., 0., 0., 0., -yaw_seed};
        }

        here_are_the_clouds_pub_.publish(here_are_the_clouds_msg);
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

    void updateAbsolutePose(int scan_index) {
        if (scan_index == 0) {
            frames_[scan_index].world_pose.position.x = 0.0;
            frames_[scan_index].world_pose.position.y = 0.0;
            frames_[scan_index].world_pose.position.z = 0.0;
            frames_[scan_index].world_pose.orientation.w = 1.0;
            frames_[scan_index].world_pose.orientation.x = 0.0;
            frames_[scan_index].world_pose.orientation.y = 0.0;
            frames_[scan_index].world_pose.orientation.z = 0.0;
            return;
        }
        int last_index = frames_[scan_index].last_scan_index;
        if (frames_.find(last_index) == frames_.end()) {
            ROS_WARN("Missing keyframe %d, cannot compute absolute pose for %d", last_index, scan_index);
            return;
        }
        // Recursively update the last keyframe's absolute pose first
        updateAbsolutePose(last_index);
        // Use sequential_keyframe_registration if available, otherwise fall back to odometry
        if (!isQuaternionZero(frames_[scan_index].sequential_keyframe_registration.orientation)) {
            frames_[scan_index].world_pose = applyTransform(
                frames_[last_index].world_pose, 
                frames_[scan_index].sequential_keyframe_registration);
        } else {
            frames_[scan_index].world_pose = applyTransform(
                frames_[last_index].world_pose, 
                frames_[scan_index].odom_constraint);
        }
    }


    bool isQuaternionZero(const geometry_msgs::Quaternion& q) {
        return q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 0.0;
    }


    geometry_msgs::Pose applyTransform(const geometry_msgs::Pose& base, 
                                    const geometry_msgs::Pose& relative) {
        geometry_msgs::Pose new_pose;

        // Convert poses to Eigen types for easier math
        Eigen::Quaterniond q_base(base.orientation.w, base.orientation.x, 
                                base.orientation.y, base.orientation.z);
        Eigen::Quaterniond q_rel(relative.orientation.w, relative.orientation.x, 
                                relative.orientation.y, relative.orientation.z);
        Eigen::Vector3d t_base(base.position.x, base.position.y, base.position.z);
        Eigen::Vector3d t_rel(relative.position.x, relative.position.y, relative.position.z);

        // Apply transformation
        Eigen::Quaterniond q_new = q_base * q_rel;
        q_new.normalize();
        Eigen::Vector3d t_new = t_base + q_base * (-1*t_rel); //sign got flipped somewhere... 

        // Convert back to geometry_msgs
        new_pose.orientation.w = q_new.w();
        new_pose.orientation.x = q_new.x();
        new_pose.orientation.y = q_new.y();
        new_pose.orientation.z = q_new.z();
        new_pose.position.x = t_new.x();
        new_pose.position.y = t_new.y();
        new_pose.position.z = t_new.z();

        // std::cout << "Base Pos: (" << t_base.transpose() << "), \n"
        //         << "Rel Pos: (" << t_rel.transpose() << "), \n "
        //         << "New Pos: (" << t_new.transpose() << ")\n";

        std::cout << "q_base: " << q_base.coeffs().transpose() << std::endl;
        std::cout << "q_rel: " << q_rel.coeffs().transpose() << std::endl;
        std::cout << "q_new: " << q_new.coeffs().transpose() << std::endl;

        return new_pose;
    }

    void publishKeyframePoses() {
        woodhouse::KeyframePoses msg;
        
        for (const auto& [scan_index, frame] : frames_) {
            woodhouse::KeyframePose kf_msg;
            kf_msg.id = scan_index;
            kf_msg.pose = frame.world_pose;  // Use the absolute pose

            msg.keyframes.push_back(kf_msg);
        }

        keyframe_pose_pub_.publish(msg);
    }

private:
    ros::Subscriber keyframe_sub_;
    ros::Subscriber get_these_clouds_sub_;
    ros::Subscriber loop_closure_sub_;
    ros::Publisher here_are_the_clouds_pub_;
    ros::Publisher keyframe_pose_pub_;


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

        csv_file << "constraint_type,scan1_index,scan2_index,position_x,position_y,position_z,orientation_x,orientation_y,orientation_z,orientation_w\n";
        csv_file.close();
        ROS_INFO("Initialized pose CSV file: %s", filename.c_str());
    }

    void appendPoseToCSV(const std::string& filename, int constraint_type, int scan1_index, int scan2_index, const geometry_msgs::Pose& pose) {
        // Open the CSV file in append mode
        std::ofstream csv_file(filename, std::ios::app);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to open pose CSV file for appending: %s", filename.c_str());
            return;
        }

        // Append the scan index and pose information
        csv_file << constraint_type << "," << scan1_index << "," << scan2_index << ","
                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
                 << pose.orientation.x << "," << pose.orientation.y << "," 
                 << pose.orientation.z << "," << pose.orientation.w << "\n";

        csv_file.close();
        // ROS_INFO("Appended pose to CSV: scan_index = %d", scan1_index);
    }




};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle nh;
    PoseGraphNode pg_node(nh);
    ros::spin();
    return 0;
}


