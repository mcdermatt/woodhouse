#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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

        gtsam::ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam_ = gtsam::ISAM2(parameters);

        prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
        odometry_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished());
        loop_closure_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());
        dense_count_ = 0;                        //counter var
        dense_count_thresh_ = 1;                 //find a new dense constraint every dense_count_thresh_ frames
        dense_constraint_start_ = 20;            //hold off on looking for dense constraints until we have this many keyframes
        dense_constraint_distance_thresh_ = 1.0; //don't find dense constraints between keyframes with indices <= this value 
    }

    void keyframeDataCallback(const woodhouse::KeyframeData::ConstPtr& msg) {
        // ROS_INFO("Received keyframe with scan_index: %d", msg->scan_index);
        // cout << msg->odom_constraint << endl;

        //add new frame to internal map
        if (frames_.find(msg->scan_index) == frames_.end()){
            Keyframe frame_i;
            frame_i.last_scan_index=msg -> last_scan_index;
            frame_i.point_cloud = msg->point_cloud;
            frame_i.odom_constraint = msg->odom_constraint;
            // Set initial world pose
            if (msg->scan_index == 0) {
                // First frame - set to identity
                frame_i.world_pose.position.x = 0.0;
                frame_i.world_pose.position.y = 0.0;
                frame_i.world_pose.position.z = 0.0;
                frame_i.world_pose.orientation.w = 1.0;
                frame_i.world_pose.orientation.x = 0.0;
                frame_i.world_pose.orientation.y = 0.0;
                frame_i.world_pose.orientation.z = 0.0;
            } else {
                // Get the previous frame's pose
                int prev_index = msg->last_scan_index;
                if (frames_.find(prev_index) != frames_.end()) {             
                    // Use the relative transform to compute this frame's world pose
                    // if (!isQuaternionZero(frame_i.sequential_keyframe_registration.orientation)) {
                    //     frame_i.world_pose = applyTransform(
                    //         frames_[prev_index].world_pose,
                    //         frame_i.sequential_keyframe_registration);
                    // } else {
                        // frame_i.world_pose = applyTransform(
                        //     frames_[prev_index].world_pose,
                        //     frame_i.odom_constraint);
                    // }
                    // Debug: just set to most recent transform as debug
                    frame_i.world_pose = frames_[prev_index].world_pose;
                } else {
                    ROS_WARN("Previous frame %d not found, setting to identity", prev_index);
                    frame_i.world_pose.position.x = 0.0;
                    frame_i.world_pose.position.y = 0.0;
                    frame_i.world_pose.position.z = 0.0;
                    frame_i.world_pose.orientation.w = 1.0;
                    frame_i.world_pose.orientation.x = 0.0;
                    frame_i.world_pose.orientation.y = 0.0;
                    frame_i.world_pose.orientation.z = 0.0;
                }
            }            


            frames_[msg->scan_index] = frame_i;
        }        

        publishKeyframePoses();

        //For debug with Jupyter Notebook: ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        // save point cloud to .csv file, titled with keyframe index
        string fn = "keyframe_" + to_string(msg->scan_index) + ".csv";
        savePointCloudToCSV(msg->point_cloud, fn);

        // // save odom constraints to text file
        appendPoseToCSV("pose_data.csv", 0, msg->last_scan_index, msg->scan_index, msg->odom_constraint);
        //hold on to constraints internally -- don't actually use odom constraints in graph soln for now
        // updateConstraints(msg->last_scan_index, msg->scan_index, msg->odom_constraint); //test uncommenting 4/13...
        // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        
        // cout << dense_count_ << endl;
        if(constraints_.size() >= dense_constraint_start_){
            if (dense_count_ == dense_count_thresh_ - 1){
                addDenseConstraint();
                // //spam dense constraints
                // for (int i = 0; i < 3; i++){
                //     addDenseConstraint();
                // }
                dense_count_ = 0;
            }else{dense_count_ += 1;}
        }

    }

    void addDenseConstraint(){
        //make sure not to go looking for dense constraints until isam_ is initialized

        // cout << "\n Constraints: " << endl;
        // for (auto row : constraints_){
        //     // for (auto e : row){
        //         // cout << e << " ";
        //     for (int e = 6; e < 8; e++){
        //         cout << row[e] << " "; 
        //     }
        //     cout << endl;
        // }
        std::map<int,int> constraint_counts;
        for (const auto& row : constraints_){
            int first = static_cast<int>(row[6]);
            constraint_counts[first]++;
        }
        // for (const auto& [idx1, count] : constraint_counts) {
        //     std::cout << "idx1 = " << idx1 << ", count = " << count << std::endl;
        // }
        int idx1;
        int idx2;
        auto candidate = findBestDenseConstraintCandidate();
        if (candidate) {
            idx1 = candidate->first;
            idx2 = candidate->second;
            dense_candidates_.push_back({idx1,idx2});//hold on to what we've tried using so far
        } else {
            std::cout << "No valid dense loop closure candidates found." << std::endl;
            return;
        }

        cout << "adding dense constraint between " << idx1 << " and " << idx2 << endl;

        //get relative transforms from iSAM graph
        vector<double> X21 = getRelativeTransformBetweenKeyframes(idx2, idx1);

        //send scan indices, initial transform, and raw point clouds to loop closer node
        woodhouse::HereAreTheClouds here_are_the_clouds_msg;
        here_are_the_clouds_msg.scan1_index = idx1;
        here_are_the_clouds_msg.scan2_index = idx2;
        here_are_the_clouds_msg.point_cloud1 = frames_[idx1].point_cloud;
        here_are_the_clouds_msg.point_cloud2 = frames_[idx2].point_cloud;
        here_are_the_clouds_msg.X0 = vector<float>{static_cast<float>(X21[0]),
                                                   static_cast<float>(X21[1]),
                                                   static_cast<float>(X21[2]),
                                                   static_cast<float>(X21[3]),
                                                   static_cast<float>(X21[4]),
                                                   static_cast<float>(X21[5])};
        here_are_the_clouds_pub_.publish(here_are_the_clouds_msg);

        //normal loop closure constraint type is flagged as 2, setting this as 3 for debug purposes
        appendPoseToCSV("pose_data.csv", 3, idx1, idx2, X21);
    }

    std::optional<std::pair<int, int>> findBestDenseConstraintCandidate(
        double search_radius = 2.0,
        int temporal_threshold = 5,      //in number of frames
        double min_yaw_diff_deg = 20.0   //NEW-- only consider keyframes with different heading direction?? 
    ) {
        std::optional<std::pair<int, int>> best_candidate;
        double best_score = std::numeric_limits<double>::max(); // lower is better

        // Step 1: Extract sorted list of frame IDs
        std::vector<int> frame_ids;
        for (const auto& [id, _] : frames_)
            frame_ids.push_back(id);
        if (frame_ids.size() < 7) //prevent segfault
            return std::nullopt;
        //skip first keyframe since it doesn't have LC, ignore the last few while graph updates
        for (size_t i = 2; i < frame_ids.size() - 5; ++i) { 
            int id1 = frame_ids[i];
            const auto& pose1 = frames_.at(id1).world_pose;
            double yaw1 = yawFromQuaternion(pose1.orientation);

            for (size_t j = i + 1; j < frame_ids.size()-4; ++j) {
                int id2 = frame_ids[j];
                const auto& pose2 = frames_.at(id2).world_pose;

                if (std::abs(id1 - id2) < temporal_threshold)
                    continue;

                double dist = euclideanDistance(pose1, pose2);
                if (dist > search_radius)
                    continue;

                if (constraintExists(id1, id2))
                    continue;

                // Compute yaw difference (in degrees)
                double yaw2 = yawFromQuaternion(pose2.orientation);
                double yaw_diff = std::fabs(yaw1 - yaw2);
                yaw_diff = std::fmod(yaw_diff, 2 * M_PI); // wrap to [0, 2pi]
                if (yaw_diff > M_PI) yaw_diff = 2 * M_PI - yaw_diff; // wrap to [0, pi]
                double yaw_diff_deg = yaw_diff * 180.0 / M_PI;

                if (yaw_diff_deg < min_yaw_diff_deg && abs(id1 - id2) < 20)
                    continue;

                // Scoring based only on distance for now
                if (dist < best_score) {
                    best_score = dist;
                    best_candidate = std::make_pair(id1, id2);
                }
            }
        }        
        return best_candidate; // std::nullopt if none found
    }

    std::vector<double> getRelativeTransformBetweenKeyframes(int keyframe1, int keyframe2) {
        if (!isam_.valueExists(keyframe1) || !isam_.valueExists(keyframe2)) {
            throw std::runtime_error("One or both keyframes do not exist in the current estimate.");
        }

        gtsam::Pose3 pose1 = isam_.calculateEstimate<gtsam::Pose3>(keyframe1);
        gtsam::Pose3 pose2 = isam_.calculateEstimate<gtsam::Pose3>(keyframe2);
        gtsam::Pose3 relative_pose = pose1.between(pose2);

        gtsam::Point3 t = relative_pose.translation();
        gtsam::Rot3 R = relative_pose.rotation();

        gtsam::Vector3 ypr = R.ypr();  // Yaw, Pitch, Roll
        double yaw = ypr(0);
        double pitch = ypr(1);
        double roll = ypr(2);

        return {t.x(), t.y(), t.z(), roll, pitch, yaw};
    }

    double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
        tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
        tf2::Matrix3x3 m(tf_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    void loopClosureCallback(const woodhouse::LoopClosed::ConstPtr& msg) {
        // cout << "Received loop closure constraint between: " << msg->scan1_index << " and " << msg->scan2_index << endl;

        // subsequent keyframes -- flag with 1
        if (msg->scan2_index - msg->scan1_index == 1){ 
            appendPoseToCSV("pose_data.csv", 1, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            updateConstraints(msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
            //hold on to SKR and associate with frame
            frames_[msg->scan1_index].sequential_keyframe_registration = msg->loop_closure_constraint;
        }
        //actual loop closure -- flag with 2
        else{
            //ignore instances where ICP diverges (generally happens in loop closure, need to maintain connection between SKRs)
            if (msg->failed_to_converge == false){
                appendPoseToCSV("pose_data.csv", 2, msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);
                updateConstraints(msg->scan1_index, msg->scan2_index, msg->loop_closure_constraint);

                //run pose graph optimization-- ideally we should only be running it in here BUT without odometry constraints GTSAM won't have a node associated with our loop closures?
                // optimizeConstraints();                
            }
        }
        optimizeConstraints(); //TODO-- do we actually need to optimize every time we add something to the graph?

    }

    void getTheseCloudsCallback(const woodhouse::GetTheseClouds::ConstPtr& msg) {
        // Print the scan_index from the received message
        // std::cout << "Pose_graph_node: publishing clouds for indices:" << msg->scan1_index << " and " << msg->scan2_index << std::endl;

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

            // cout << "seeding roll: " << rpy[0] << " pitch: " << rpy[1] << " yaw: " << rpy[2] << endl;

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
        // ROS_INFO("PointCloud saved to: %s", filename.c_str());
    }

    void optimizeConstraints() {
        try {           
            // Initialize ISAM2 and add prior only on first run            
            if (first_run_) {               
                gtsam::NonlinearFactorGraph prior_graph;
                gtsam::Values prior_values;
                
                gtsam::Pose3 prior_pose(gtsam::Rot3::identity(), gtsam::Point3(0, 0, 0));
                prior_graph.add(gtsam::PriorFactor<gtsam::Pose3>(1, prior_pose, prior_noise_));
                prior_values.insert(1, prior_pose);                
                isam_.update(prior_graph, prior_values);
                first_run_ = false;
            }
            
            // Process all constraints in a single batch
            gtsam::NonlinearFactorGraph new_graph;
            gtsam::Values new_values;
            gtsam::Values current_estimate;
            
            // Only calculate current estimate once if needed
            bool need_current_estimate = false;
            for (const auto& constraint : constraints_new_) {
                int keyframe2 = static_cast<int>(constraint[7]);
                if (!isam_.valueExists(keyframe2)) {
                    need_current_estimate = true;
                    break;
                }
            }
            
            if (need_current_estimate) {
                current_estimate = isam_.calculateEstimate();
            }
            
            // Add each constraint to the graph
            for (const auto& constraint : constraints_new_) {
                // Extract constraint data (with sign correction)
                double x = -constraint[0];
                double y = -constraint[1];
                double yaw = -constraint[5];

                // full 6DOF motion
                double z = -constraint[2];
                double roll = -constraint[3];
                double pitch = -constraint[4];

                // // planar motion 
                // double z = 0.;
                // double roll = 0.;
                // double pitch = 0.;
                
                int keyframe1 = static_cast<int>(constraint[6]);
                int keyframe2 = static_cast<int>(constraint[7]);
                
                // Create the relative pose transformation
                gtsam::Rot3 rotation = gtsam::Rot3::Ypr(yaw, pitch, roll);

                gtsam::Point3 translation(x, y, z);
                gtsam::Pose3 relative_pose(rotation, translation);
                
                // Add between factor
                if (keyframe1 - keyframe2 > 1){
                    // use larger noise for loop closure constraints
                    new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(keyframe1, keyframe2, relative_pose, loop_closure_noise_));
                }else{
                    new_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(keyframe1, keyframe2, relative_pose, odometry_noise_));
                }
                
                // Add initial values for new keyframes
                if (!isam_.valueExists(keyframe1)) {
                    // If the first pose in a constraint doesn't exist, initialize it at origin
                    // This should rarely happen as keyframe1 is usually already in the graph
                    new_values.insert(keyframe1, gtsam::Pose3());
                }
                
                if (!isam_.valueExists(keyframe2)) {
                    // Initialize the second keyframe based on the relative transformation from the first
                    gtsam::Pose3 prev_pose = current_estimate.at<gtsam::Pose3>(keyframe1);
                    new_values.insert(keyframe2, prev_pose.compose(relative_pose));
                }
            }
            
            // Update ISAM2 with all new factors and values at once
            isam_.update(new_graph, new_values);
            isam_.update(); // Perform final optimization

            // Clear the new constraints after processing
            constraints_new_.clear();
            
            gtsam::Values result = isam_.calculateEstimate();
            // std::cout << "Optimization complete with " << result.size() << " poses" << std::endl;
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
                    // std::cout << "frame " << keyframe_idx << " world_pose " << translation << std::endl;
                }
            }
            //TODO for elements in frames_ but not in graph yet, use odometry to update absolute poses? 
            // After updating all optimized frames
            int max_optimized_key = -1;
            for (const auto& key_value : result) {
                int keyframe_idx = static_cast<int>(key_value.key);
                if (keyframe_idx > max_optimized_key) {
                    max_optimized_key = keyframe_idx;
                }
            }

            // Fill in frames not optimized yet
            for (const auto& kv : frames_) {
                int idx = kv.first;
                if (!result.exists(idx)) {
                    // Use last optimized frame’s pose if possible
                    if (frames_.find(max_optimized_key) != frames_.end()) {
                        frames_[idx].world_pose = frames_[max_optimized_key].world_pose;
                        // std::cout << "frame " << idx << " not in optimized result, defaulting to last known pose" << std::endl;
                    }
                }
            }

        } catch (const std::exception& e) {
            std::cerr << "GTSAM Exception: " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "Unknown exception in optimizeConstraints()" << std::endl;
        }
    }

    bool isQuaternionZero(const geometry_msgs::Quaternion& q) {
        // A quaternion is zero if all components are zero or very close to zero
        const double epsilon = 1e-6;
        return (std::abs(q.w) < epsilon && 
                std::abs(q.x) < epsilon && 
                std::abs(q.y) < epsilon && 
                std::abs(q.z) < epsilon);
    }

    geometry_msgs::Pose applyTransform(const geometry_msgs::Pose& base, 
                                    const geometry_msgs::Pose& relative) {

        // Apply the same sign flips as in optimizeConstraints()
        geometry_msgs::Pose flipped_relative = relative;
        flipped_relative.position.x = -relative.position.x;
        flipped_relative.position.y = -relative.position.y;
        flipped_relative.position.z = -relative.position.z;
        
        // For quaternions, negating x, y, z components is equivalent to negating roll, pitch, yaw
        // (while keeping w the same)
        flipped_relative.orientation.x = -relative.orientation.x;
        flipped_relative.orientation.y = -relative.orientation.y;
        flipped_relative.orientation.z = -relative.orientation.z;
        
        // Create GTSAM Pose3 objects
        gtsam::Rot3 base_rot = gtsam::Rot3::Quaternion(
            base.orientation.w, 
            base.orientation.x, 
            base.orientation.y, 
            base.orientation.z);
        gtsam::Point3 base_trans(base.position.x, base.position.y, base.position.z);
        gtsam::Pose3 base_pose(base_rot, base_trans);
        
        // gtsam::Rot3 rel_rot = gtsam::Rot3::Quaternion(
        //     relative.orientation.w, 
        //     relative.orientation.x, 
        //     relative.orientation.y, 
        //     relative.orientation.z);
        // gtsam::Point3 rel_trans(relative.position.x, relative.position.y, relative.position.z);
        //flip sign to match optimizeConstraints()
        gtsam::Rot3 rel_rot = gtsam::Rot3::Quaternion(
            flipped_relative.orientation.w, 
            flipped_relative.orientation.x, 
            flipped_relative.orientation.y, 
            flipped_relative.orientation.z);
        gtsam::Point3 rel_trans(flipped_relative.position.x, flipped_relative.position.y, flipped_relative.position.z);
        gtsam::Pose3 rel_pose(rel_rot, rel_trans);
        
        // Use GTSAM's compose method - exactly the same as in optimizeConstraints()
        gtsam::Pose3 result = base_pose.compose(rel_pose);
        
        // Convert back to geometry_msgs::Pose
        geometry_msgs::Pose new_pose;
        new_pose.position.x = result.translation().x();
        new_pose.position.y = result.translation().y();
        new_pose.position.z = result.translation().z();
        
        gtsam::Quaternion q = result.rotation().toQuaternion();
        new_pose.orientation.w = q.w();
        new_pose.orientation.x = q.x();
        new_pose.orientation.y = q.y();
        new_pose.orientation.z = q.z();
        
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
    vector<vector<double>> constraints_; // [x, y, z, r, p, y, keyframe1, keyframe2] //all constraints
    vector<vector<double>> constraints_new_; // [x, y, z, r, p, y, keyframe1, keyframe2] //new constraints to be added to graph
    gtsam::SharedNoiseModel odometry_noise_;
    gtsam::SharedNoiseModel loop_closure_noise_;
    gtsam::SharedNoiseModel prior_noise_;
    int dense_count_;
    int dense_count_thresh_;
    int dense_constraint_start_;
    double dense_constraint_distance_thresh_;
    gtsam::ISAM2 isam_;
    bool first_run_ = true;
    vector<vector<int>> dense_candidates_; //to track what we've attempted to use for dense loop closure

    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
    }

    int getRandomInt(int n) {
        std::random_device rd;                      // seed
        std::mt19937 gen(rd());                     // Mersenne Twister RNG
        std::uniform_int_distribution<> dist(1, n); // inclusive range [1, n]
        return dist(gen);
    }

    // Utility: compute 3D Euclidean distance between two geometry_msgs::Pose positions
    double euclideanDistance(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) {
        double dx = a.position.x - b.position.x;
        double dy = a.position.y - b.position.y;
        double dz = a.position.z - b.position.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Utility: check if a constraint already exists between two keyframes
    //TODO-- if repeatedly sampling the same keyframe pair that fails to converge over and over again
    //       make additional vector<int> to track failures and check it here...
    bool constraintExists(int id1, int id2) {
        for (const auto& c : constraints_) {
            int c1 = static_cast<int>(c[6]);
            int c2 = static_cast<int>(c[7]);
            if ((c1 == id1 && c2 == id2) || (c1 == id2 && c2 == id1)) {
                return true;
            }
        }
        // Check in dense_candidates_
        for (const auto& pair : dense_candidates_) {
            if ((pair[0] == id1 && pair[1] == id2) || (pair[0] == id2 && pair[1] == id1)) {
                return true;
            }
        }

        return false;
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
        // ROS_INFO("Initialized pose CSV file: %s", filename.c_str());
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

    //overloading for cases when we just get the xyzrpy back from the isam_ graph 
    void appendPoseToCSV(const std::string& filename, int constraint_type, int scan1_index, int scan2_index, vector<double> X) {
        // Open the CSV file in append mode
        std::ofstream csv_file(filename, std::ios::app);
        if (!csv_file.is_open()) {
            ROS_ERROR("Failed to open pose CSV file for appending: %s", filename.c_str());
            return;
        }

        geometry_msgs::Pose pose = vectorToPose(X); 

        // Append the scan index and pose information
        csv_file << constraint_type << "," << scan1_index << "," << scan2_index << ","
                 << pose.position.x << "," << pose.position.y << "," << pose.position.z << ","
                 << pose.orientation.x << "," << pose.orientation.y << "," 
                 << pose.orientation.z << "," << pose.orientation.w << "\n";

        csv_file.close();
        // ROS_INFO("Appended pose to CSV: scan_index = %d", scan1_index);
    }

    geometry_msgs::Pose vectorToPose(const std::vector<double>& vec) {
        if (vec.size() != 6) {
            throw std::runtime_error("Expected vector of size 6: [x, y, z, roll, pitch, yaw]");
        }

        geometry_msgs::Pose pose;
        pose.position.x = vec[0];
        pose.position.y = vec[1];
        pose.position.z = vec[2];

        tf2::Quaternion q;
        q.setRPY(vec[3], vec[4], vec[5]);  // Roll, Pitch, Yaw
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
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
        constraints_new_.push_back(new_constraint);

        //sort by first index (index 6)
        std::sort(constraints_.begin(), constraints_.end(), 
            [](const std::vector<double>& a, const std::vector<double>& b) {
                return a[6] < b[6];
            });
        std::sort(constraints_new_.begin(), constraints_new_.end(), 
            [](const std::vector<double>& a, const std::vector<double>& b) {
                return a[6] < b[6];
            });


        // cout << "\n Constraints: " << endl;
        // for (auto row : constraints_){
        //     // for (auto e : row){
        //         // cout << e << " ";
        //     for (int e = 6; e < 8; e++){
        //         cout << row[e] << " "; 
        //     }
        //     cout << endl;
        // }
    }

};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_graph_node");
    ros::NodeHandle nh;
    PoseGraphNode pg_node(nh);
    ros::spin();
    return 0;
}


