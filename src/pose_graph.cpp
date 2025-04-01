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
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h> 
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include "woodhouse/KeyframeData.h"
#include "woodhouse/GetTheseClouds.h"
#include "woodhouse/HereAreTheClouds.h"
#include "woodhouse/LoopClosed.h"
#include "woodhouse/KeyframePose.h"
#include "woodhouse/KeyframePoses.h"
#include <map>

using namespace std;
using namespace Eigen;

// //not sure if this is going to help
// #define EIGEN_DONT_ALIGN_STATICALLY
// #define GTSAM_ALLOW_DEPRECATED_SINCE_V4

struct Keyframe {
    int scan_index;                                       // Index of the scan
    int last_scan_index;                                  // Last keyframe index for relative odometry constraint
    sensor_msgs::PointCloud2 point_cloud;                 // Point cloud of the keyframe
    std::vector<float> scan_context;                      // Scan context feature vector
    std::vector<float> ring_keys;                         // Ring key feature vector
    geometry_msgs::Pose odom_constraint;                  // Odometry estimate (dead reckoning from registering every raw point cloud )
    geometry_msgs::Pose sequential_keyframe_registration; //directly using ICP to align subsequent keyframes (less drift than odom)
    geometry_msgs::Pose world_pose;                       // Absolute pose
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

        // cout << "attempting to spin up gtsam" << endl;

        prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
        odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

    }

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
            // Explicitly initialize world_pose to zero quat as flag for updateAbsolutePose
            frame_i.world_pose.position.x = 0.0;
            frame_i.world_pose.position.y = 0.0;
            frame_i.world_pose.position.z = 0.0;
            frame_i.world_pose.orientation.w = 0.0;
            frame_i.world_pose.orientation.x = 0.0;
            frame_i.world_pose.orientation.y = 0.0;
            frame_i.world_pose.orientation.z = 0.0;
            // Initialize sequential_keyframe_registration to zero quaternion
            // so we know it hasn't been set yet
            // frame_i.sequential_keyframe_registration.orientation.w = 0.0;
            // frame_i.sequential_keyframe_registration.orientation.x = 0.0;
            // frame_i.sequential_keyframe_registration.orientation.y = 0.0;
            // frame_i.sequential_keyframe_registration.orientation.z = 0.0;
            frames_[msg->scan_index] = frame_i;

            // DEAD RECKONING ONLY via sequential keyframe registration (if available, otherwise fall back to odometry)
            // updateAbsolutePose(msg->scan_index); 
        }        

        publishKeyframePoses();

        //For debug with Jupyter Notebook: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // save point cloud to .csv file, titled with keyframe index
        string fn = "keyframe_" + to_string(msg->scan_index) + ".csv";
        savePointCloudToCSV(msg->point_cloud, fn);

        // // save odom constraints to text file
        appendPoseToCSV("pose_data.csv", 0, msg->last_scan_index, msg->scan_index, msg->odom_constraint);
        //hold on to constraints internally -- don't actually use odom constraints in graph soln for now
        // updateConstraints(msg->last_scan_index, msg->scan_index, msg->odom_constraint);
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    }

    void loopClosureCallback(const woodhouse::LoopClosed::ConstPtr& msg) {
        cout << "Received loop closure constraint between: " << msg->scan1_index << " and " << msg->scan2_index << endl;

        // subsequent keyframes -- flag with 1
        if (msg->scan2_index - msg->scan1_index == 1){ 
            appendPoseToCSV("pose_data.csv", 1, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            updateConstraints(msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            //hold on to SKR and associate with frame
            frames_[msg->scan1_index].sequential_keyframe_registration = msg->loop_closure_constraint;
        }
        //actual loop closure -- flag with 2
        else{
            //ignore instances where ICP diverges
            if (msg->failed_to_converge == false){
                appendPoseToCSV("pose_data.csv", 2, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
                updateConstraints(msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);

                //run pose graph optimization-- ideally we should only be running it in here BUT without odometry constraints GTSAM won't have a node associated with our loop closures?
                // optimizeConstraints();

                //TODO -- after pose graph has convergd a bit try to register rearby point clouds that aren't in constraints yet
                
            }
        }
        optimizeConstraints(); //TODO-- do we actually need to optimize every time we add something to the graph?

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

    // TODO-- integrate raw Sequential Keyframe Registrations back until we reach parts of optimized graph
    void updateAbsolutePose(int scan_index) {
        // Case 1: This is the origin keyframe
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
        
        // Check if this frame already has a valid pose from GTSAM optimization
        if (!isQuaternionZero(frames_[scan_index].world_pose.orientation) && 
            (frames_[scan_index].world_pose.position.x != 0.0 || 
            frames_[scan_index].world_pose.position.y != 0.0 || 
            frames_[scan_index].world_pose.position.z != 0.0)) {
            // Already has a valid optimized pose from GTSAM, no need to update
            return;
        }
        
        // Frame needs initial pose estimate via dead reckoning
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

    void optimizeConstraints() {
        try {
            std::cout << "Using iSAM2 to perform graph optimization" << std::endl;
            
            // Initialize ISAM2 and add prior only on first run
            static bool first_run = true;
            static gtsam::ISAM2 isam;
            
            if (first_run) {
                // Create ISAM2 with parameters
                gtsam::ISAM2Params parameters;
                parameters.relinearizeThreshold = 0.01;
                parameters.relinearizeSkip = 1;
                isam = gtsam::ISAM2(parameters);
                
                // Add prior factor for the first pose
                gtsam::NonlinearFactorGraph prior_graph;
                gtsam::Values prior_values;
                
                gtsam::Pose3 prior_pose(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));
                prior_graph.add(gtsam::PriorFactor<gtsam::Pose3>(1, prior_pose, prior_noise_));
                prior_values.insert(1, prior_pose);
                
                isam.update(prior_graph, prior_values);
                first_run = false;
            }
            
            // Process all constraints in a single batch
            gtsam::NonlinearFactorGraph new_graph;
            gtsam::Values new_values;
            gtsam::Values current_estimate;
            
            // Only calculate current estimate once if needed
            bool need_current_estimate = false;
            for (const auto& constraint : constraints_) {
                int keyframe2 = static_cast<int>(constraint[7]);
                if (!isam.valueExists(keyframe2)) {
                    need_current_estimate = true;
                    break;
                }
            }
            
            if (need_current_estimate) {
                current_estimate = isam.calculateEstimate();
            }
            
            // Add each constraint to the graph
            for (const auto& constraint : constraints_) {
                // Extract constraint data (with sign correction)
                double x = -constraint[0];
                double y = -constraint[1];
                double z = -constraint[2];
                double roll = -constraint[3];
                double pitch = -constraint[4];
                double yaw = -constraint[5];
                int keyframe1 = static_cast<int>(constraint[6]);
                int keyframe2 = static_cast<int>(constraint[7]);
                
                // Create the relative pose transformation
                gtsam::Rot3 rotation = gtsam::Rot3::Ypr(yaw, pitch, roll);
                gtsam::Point3 translation(x, y, z);
                gtsam::Pose3 relative_pose(rotation, translation);
                
                // Add between factor
                new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(keyframe1, keyframe2, relative_pose, odometry_noise_));
                
                // Add initial values for new keyframes
                if (!isam.valueExists(keyframe1)) {
                    // If the first pose in a constraint doesn't exist, initialize it at origin
                    // This should rarely happen as keyframe1 is usually already in the graph
                    new_values.insert(keyframe1, gtsam::Pose3());
                }
                
                if (!isam.valueExists(keyframe2)) {
                    // Initialize the second keyframe based on the relative transformation from the first
                    gtsam::Pose3 prev_pose = current_estimate.at<gtsam::Pose3>(keyframe1);
                    new_values.insert(keyframe2, prev_pose.compose(relative_pose));
                }
            }
            
            // Update ISAM2 with all new factors and values at once
            isam.update(new_graph, new_values);
            isam.update(); // Perform final optimization
            
            // // Clear the constraints after processing
            // constraints_.clear();
            
            gtsam::Values result = isam.calculateEstimate();
            std::cout << "Optimization complete with " << result.size() << " poses" << std::endl;
            // Update world_pose for each keyframe in frames_
            for (const auto& key_value : result) {
                gtsam::Key key = key_value.key;
                int keyframe_idx = static_cast<int>(key);                
                // Check if this keyframe exists in our frames_ map
                if (frames_.find(keyframe_idx) != frames_.end()) {
                    // Get the optimized pose from GTSAM
                    gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(key);
                    // Extract translation
                    gtsam::Point3 translation = optimized_pose.translation();
                    frames_[keyframe_idx].world_pose.position.x = translation.x();
                    frames_[keyframe_idx].world_pose.position.y = translation.y();
                    frames_[keyframe_idx].world_pose.position.z = translation.z();                    
                    // Extract rotation as quaternion
                    gtsam::Rot3 rotation = optimized_pose.rotation();
                    gtsam::Quaternion quat = rotation.toQuaternion();
                    frames_[keyframe_idx].world_pose.orientation.w = quat.w();
                    frames_[keyframe_idx].world_pose.orientation.x = quat.x();
                    frames_[keyframe_idx].world_pose.orientation.y = quat.y();
                    frames_[keyframe_idx].world_pose.orientation.z = quat.z();
                    std::cout << "frame " << keyframe_idx << " world_pose " << translation << std::endl;
                }
            }
            //TODO for elements in frames_ but not in graph yet, use odometry to update absolute poses? 

        } catch (const std::exception& e) {
            std::cerr << "GTSAM Exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown exception in optimizeConstraints()" << std::endl;
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
    map<int, Keyframe> frames_;
    vector<vector<double>> constraints_; // [x, y, z, r, p, y, keyframe1, keyframe2]
    gtsam::SharedNoiseModel odometry_noise_;
    gtsam::SharedNoiseModel prior_noise_;

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

    void updateConstraints(int scan1_index, int scan2_index, const geometry_msgs::Pose& pose){
        double x = pose.position.x;
        double y = pose.position.y;
        double z = pose.position.z;

        tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

        vector<double> new_constraint = {x,
                                         y, 
                                         z, 
                                         roll, 
                                         pitch, 
                                         yaw, 
                                         static_cast<double>(scan1_index),
                                         static_cast<double>(scan2_index)}; 
        // zero out rpy for first constraint (rare issue in odom can cause NaN values here) -- SR not run between first keyframes anyways(?) 
        // if (constraints_.size() == 0){
        //     new_constraint[4] = 0;
        //     new_constraint[5] = 0;
        //     new_constraint[6] = 0;
        // }
        constraints_.push_back(new_constraint);

        //sort by first index (index 6)
        std::sort(constraints_.begin(), constraints_.end(), 
            [](const std::vector<double>& a, const std::vector<double>& b) {
                return a[6] < b[6];
            });

        cout << "\n Constraints: " << endl;
        for (auto row : constraints_){
            // for (auto e : row){
                // cout << e << " ";
            for (int e = 6; e < 8; e++){
                cout << row[e] << " "; 
            }
            cout << endl;
        }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle nh;
    PoseGraphNode pg_node(nh);
    ros::spin();
    return 0;
}


