#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>  // Add this line for tf2_ros::Buffer
#include <tf2/LinearMath/Quaternion.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
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
#include <map>
#include "icet.h"
#include "utils.h"
#include <unordered_map>

using namespace std;
using namespace Eigen;

// Subscribes to "here are the clouds" topic and runs ICET point cloud registration
// publishes a 6DOF transform relatinig the two provided point clouds to one another 

class LoopCloserNode {
public:
    LoopCloserNode(ros::NodeHandle& nh){
        // Set up ROS subscribers and publishers
        here_are_the_clouds_sub_ = nh.subscribe("/here_are_the_clouds", 10, &LoopCloserNode::hereAreTheCloudsCallback, this);
        loop_closure_constraint_pub_ =  nh.advertise<woodhouse::LoopClosed>("/loop_closure_constraint", 10);
        debug_cloud1_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud1", 10);
        debug_cloud2_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud2", 10);

        // ros::Rate rate(50);

    }

    void hereAreTheCloudsCallback(const woodhouse::HereAreTheClouds::ConstPtr& msg) {

        // Print the scan_index from the received message
        std::cout << "loop_closer_node: registering PC's: " << msg->scan1_index << " and " << msg->scan2_index << std::endl;

        // extract point clouds and convert PointCloud2 to PCL PointCloud
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msg->point_cloud1, *pcl_cloud1);
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> pcl_cloud2(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(msg->point_cloud2, *pcl_cloud2);

        //convert PCL to eigen matrix
        Eigen::MatrixXf pc1_matrix = convertPCLtoEigen(pcl_cloud1);
        Eigen::MatrixXf pc2_matrix = convertPCLtoEigen(pcl_cloud2);

        // Filter out points less than distance 'd' from the origin
        float minD = 1.;
        vector<int> not_too_close_idxs1;
        for (int i = 0; i < pc1_matrix.rows(); i++){
            float distance = pc1_matrix.row(i).norm();
            if (distance > minD){
                not_too_close_idxs1.push_back(i);
            }
        }
        Eigen::MatrixXf filtered_pcl_matrix1(not_too_close_idxs1.size(), 3);
        for (std::size_t i = 0; i < not_too_close_idxs1.size(); i++){
            filtered_pcl_matrix1.row(i) = pc1_matrix.row(not_too_close_idxs1[i]);
        }
        vector<int> not_too_close_idxs2;
        for (int i = 0; i < pc2_matrix.rows(); i++){
            float distance = pc2_matrix.row(i).norm();
            if (distance > minD){
                not_too_close_idxs2.push_back(i);
            }
        }
        Eigen::MatrixXf filtered_pcl_matrix2(not_too_close_idxs2.size(), 3);
        for (std::size_t i = 0; i < not_too_close_idxs2.size(); i++){
            filtered_pcl_matrix2.row(i) = pc2_matrix.row(not_too_close_idxs2[i]);
        }

        pc1_matrix = filtered_pcl_matrix1;
        pc2_matrix = filtered_pcl_matrix2;

        //temporary fix
        if (pc1_matrix.hasNaN() == true || pc2_matrix.hasNaN() == true){
            cout << "found a NaN" << endl;
            return;
        }

        cout << "1: " << pc1_matrix.size() << " 2: " << pc2_matrix.size() << endl;

        if (pc1_matrix.size() > 1000 && pc2_matrix.size() > 1000){
            //create a loop closure msg (to send back to pose graph node)
            woodhouse::LoopClosed loop_closure_msg;

            //remove ground plane in eigen (PCL 1.10 unstable with noetic on my laptop?)
            float threshold = 0.15;
            pc1_matrix = removeGroundPlaneInEigen(pc1_matrix, threshold);
            pc2_matrix = removeGroundPlaneInEigen(pc2_matrix, threshold);

            float voxel_size_coarse = 0.5;  // 0.5 best so far...
            float voxel_size_fine = 0.2;
            Eigen::MatrixXf pc1_matrix_coarse = downsampleCloudEigen(pc1_matrix, voxel_size_coarse);
            Eigen::MatrixXf pc2_matrix_coarse = downsampleCloudEigen(pc2_matrix, voxel_size_coarse);
            Eigen::MatrixXf pc1_matrix_fine = downsampleCloudEigen(pc1_matrix, voxel_size_fine);
            Eigen::MatrixXf pc2_matrix_fine = downsampleCloudEigen(pc2_matrix, voxel_size_fine);

            //RUN ICP instead of ICET (more robust with perspective shift) ~~~~~~~~~~~
            // Initialize ICP
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

            // Create point clouds with make_shared
            auto cloud_source_coarse = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto cloud_target_coarse = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto cloud_source_fine = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto cloud_target_fine = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

            // Convert from Eigen
            *cloud_source_coarse = *eigenToPCL(pc1_matrix_coarse);
            *cloud_target_coarse = *eigenToPCL(pc2_matrix_coarse);
            *cloud_source_fine = *eigenToPCL(pc1_matrix_fine);
            *cloud_target_fine = *eigenToPCL(pc2_matrix_fine);

            //for debug-- publish cloud showing downsampled and ground plane-less cloud
            sensor_msgs::PointCloud2 cloud1_msg;
            pcl::toROSMsg(*cloud_source_fine, cloud1_msg);
            cloud1_msg.header.stamp = ros::Time::now();
            cloud1_msg.header.frame_id = "map"; // Use the correct frame
            debug_cloud1_pub_.publish(cloud1_msg);

            sensor_msgs::PointCloud2 cloud2_msg;
            pcl::toROSMsg(*cloud_target_fine, cloud2_msg);
            cloud2_msg.header.stamp = ros::Time::now();
            cloud2_msg.header.frame_id = "map"; // Use the correct frame
            debug_cloud2_pub_.publish(cloud2_msg);

            if (!cloud_source_coarse || cloud_source_coarse->empty()) {
                std::cerr << "Error: downsampled cloud_source_coarse is null or empty!" << std::endl;
                return;
            }
            if (!cloud_target_coarse || cloud_target_coarse->empty()) {
                std::cerr << "Error: downsampled cloud_target_coarse is null or empty!" << std::endl;
                return;
            }

            icp.setInputSource(cloud_source_coarse);
            icp.setInputTarget(cloud_target_coarse);

            icp.setTransformationEpsilon(1e-8);     // Convergence criteria
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setMaximumIterations(10);

            //print out seeded initial transform (for debug)~~~~~
            Eigen::VectorXf X0(6);
            X0 << -1*msg->X0[0], -1*msg->X0[1], -1*msg->X0[2], msg->X0[3], msg->X0[4], msg->X0[5]; 
            cout << "X0: " << X0.transpose() << endl;

            // Convert X0 (6x1) to a 4x4 transformation matrix
            Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
            initial_guess.block<3,1>(0,3) << msg->X0[0], msg->X0[1], msg->X0[2]; // Translation
            // initial_guess.block<3,1>(0,3) << 0., 0., 0.; // Translation
            Eigen::Matrix3f rot;
            rot = Eigen::AngleAxisf(msg->X0[3], Eigen::Vector3f::UnitX()) *  // Roll
                Eigen::AngleAxisf(msg->X0[4], Eigen::Vector3f::UnitY()) *  // Pitch
                Eigen::AngleAxisf(msg->X0[5], Eigen::Vector3f::UnitZ());   // Yaw
            initial_guess.block<3,3>(0,0) = rot;

            double max_corr_dist;
            // provide finer initial max correspondance distnance when we're aligning neighboring keyframes
            // (should have a relatively good initial registration from odometry)
            if (msg->scan2_index - msg->scan1_index == 1){
                max_corr_dist = 0.5;
            } else{
            //provide coarser initial max correspondance distance when we are doing loop closure
                max_corr_dist = 3.0;
            }

            Eigen::Matrix4f final_transform = initial_guess;
            Eigen::VectorXf X(6);

            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            //coarse to fine ICP
            for (int i = 0; i < 5; i++){
                //hold off on using a finer version of each pc until we've converged a bit
                if (i >= 3){
                    icp.setInputSource(cloud_source_fine);
                    icp.setInputTarget(cloud_target_fine);
                }

                icp.setMaxCorrespondenceDistance(max_corr_dist);
                aligned_cloud->clear(); // <- THIS WAS THE PROBLEM??!!
                icp.align(*aligned_cloud, final_transform);

                // Check if ICP diverged
                if (!icp.hasConverged()) {
                    X << 0., 0., 0., 0., 0., 0.; 
                    std::cout << "ICP did not converge!" << std::endl;
                    loop_closure_msg.failed_to_converge = true;
                    loop_closure_msg.scan1_index = msg->scan1_index;
                    loop_closure_msg.scan2_index = msg->scan2_index;
                    loop_closure_msg.loop_closure_constraint.position.x = 0.;
                    loop_closure_msg.loop_closure_constraint.position.y = 0.;
                    loop_closure_msg.loop_closure_constraint.position.z = 0.;
                    loop_closure_msg.loop_closure_constraint.orientation.x = 0.;
                    loop_closure_msg.loop_closure_constraint.orientation.y = 0.;
                    loop_closure_msg.loop_closure_constraint.orientation.z = 0.;
                    loop_closure_msg.loop_closure_constraint.orientation.w = 1.;
                    loop_closure_constraint_pub_.publish(loop_closure_msg);
                    return;
                }

                //coarse to fine registration
                std::cout << "ICP converged, score: " << icp.getFitnessScore() << std::endl;

                // Extract transformation matrix
                Eigen::Matrix4f transformation = icp.getFinalTransformation();
                // std::cout << "ICP Transformation Matrix: \n" << transformation << std::endl;

                // Extract translation (X, Y, Z)
                Eigen::Vector3f translation = transformation.block<3,1>(0,3);
                
                // Extract rotation (convert to roll-pitch-yaw)
                Eigen::Matrix3f rotation = transformation.block<3,3>(0,0);
                double roll  = atan2(rotation(2,1), rotation(2,2));
                double pitch = asin(-rotation(2,0));
                double yaw   = atan2(rotation(1,0), rotation(0,0));

                // Store the results in X (similar to ICET output)
                X << translation(0), translation(1), translation(2), roll, pitch, yaw;

                // std::cout << "Estimated Pose (ICP) at iteration " << i << ":  " << X.transpose() << std::endl;

                // Update transform and decrease correspondence threshold
                final_transform = icp.getFinalTransformation();
                max_corr_dist *= 0.625;  // Reduce threshold gradually
            }
            std::cout << "X = " << X.transpose() << std::endl;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            //one way distances
            // double alignment_fitness = icp.getFitnessScore(); 
            // Bidirectional distances
            double forward_fitness = icp.getFitnessScore(2.0);
            icp.setInputSource(cloud_target_fine);
            icp.setInputTarget(aligned_cloud);
            double reverse_fitness = icp.getFitnessScore(2.0);
            double alignment_fitness = (forward_fitness + reverse_fitness)/2;
            // test for inlier ratio as well
            // float inlier_calc_max_dist = 0.1; //tough to get a static value that works well for both indoor and outdoor scenes
            float inlier_calc_max_dist = 0.027f * average_range(cloud_target_fine); //scale in proportion to average range in new point clouds
            //best trial with 0.03, but very unstable there...
            cout << "inlier_calc_max_dist: " << inlier_calc_max_dist << endl;

            float inlier_ratio = computeInlierRatio(aligned_cloud, cloud_target_fine, inlier_calc_max_dist);
            cout << "inlier_ratio: " << inlier_ratio << endl;

            float inlier_ratio_thresh = 0.35; //was 0.2 (indoor) (higher harder)
            double alignment_thresh = 0.30; //was 0.5 (indoor) (lower harder)

            loop_closure_msg.failed_to_converge = false;
            if (alignment_fitness > alignment_thresh || inlier_ratio < inlier_ratio_thresh){
                loop_closure_msg.failed_to_converge = true;
            }
            loop_closure_msg.scan1_index = msg->scan1_index;
            loop_closure_msg.scan2_index = msg->scan2_index;
            loop_closure_msg.loop_closure_constraint.position.x = X[0];
            loop_closure_msg.loop_closure_constraint.position.y = X[1];
            loop_closure_msg.loop_closure_constraint.position.z = X[2];
            tf2::Quaternion q;
            q.setRPY(X[3],X[4],X[5]);
            loop_closure_msg.loop_closure_constraint.orientation.x = q.x();
            loop_closure_msg.loop_closure_constraint.orientation.y = q.y();
            loop_closure_msg.loop_closure_constraint.orientation.z = q.z();
            loop_closure_msg.loop_closure_constraint.orientation.w = q.w();
            std::cout << "About to publish loop_closure_msg..." << std::endl;
            loop_closure_constraint_pub_.publish(loop_closure_msg);
            std::cout << "Successfully published loop_closure_msg" << std::endl;
        }        
    }

    float estimateCloudScale(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cloud, min_pt, max_pt);
        float dx = max_pt.x - min_pt.x;
        float dy = max_pt.y - min_pt.y;
        float dz = max_pt.z - min_pt.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz); // Diagonal length
    }
    float average_range(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        float total = 0.0f;
        for (const auto& pt : cloud->points) {
            total += std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        }
        return total / cloud->size();
    }


    float computeInlierRatio(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_source,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                            float max_dist = 0.5f){
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(target);

        int inlier_count = 0;
        std::vector<int> indices(1);
        std::vector<float> sqr_dists(1);

        for (const auto& pt : transformed_source->points)
        {
            if (kdtree.nearestKSearch(pt, 1, indices, sqr_dists) > 0)
            {
                if (std::sqrt(sqr_dists[0]) < max_dist)
                    inlier_count++;
            }
        }

        return static_cast<float>(inlier_count) / static_cast<float>(transformed_source->size());
    }


    bool containsNaN(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        for (const auto& point : cloud->points) {
            if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
                return true;
            }
        }
        return false;
    }

    // Function to calculate the distance of a point to a plane
    float pointToPlaneDistance(const Eigen::Vector3f& point, const Eigen::Vector4f& plane) {
        // Plane equation: ax + by + cz + d = 0
        return std::abs(plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3])
            / std::sqrt(plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
    }

    // RANSAC-based ground plane removal
    Eigen::MatrixXf removeGroundPlaneInEigen(const Eigen::MatrixXf& point_cloud, float distance_threshold = 0.15) {
        std::vector<Eigen::Vector3f> points;
        for (int i = 0; i < point_cloud.rows(); ++i) {
            points.push_back(Eigen::Vector3f(point_cloud(i, 0), point_cloud(i, 1), point_cloud(i, 2)));
        }

        // RANSAC to fit a plane
        Eigen::Vector4f best_plane; // Plane parameters [a, b, c, d]
        int max_inliers = 0;
        std::vector<int> best_inlier_indices;  // Store indices instead of points

        // Initialize random seed
        srand(time(nullptr));

        for (int iter = 0; iter < 100; ++iter) {
            // Randomly select three points
            int idx1 = rand() % points.size();
            int idx2 = rand() % points.size();
            int idx3 = rand() % points.size();

            // Skip if we selected the same point twice
            if (idx1 == idx2 || idx2 == idx3 || idx1 == idx3) {
                continue;
            }

            Eigen::Vector3f p1 = points[idx1];
            Eigen::Vector3f p2 = points[idx2];
            Eigen::Vector3f p3 = points[idx3];

            // Compute plane normal using cross product
            Eigen::Vector3f v1 = p2 - p1;
            Eigen::Vector3f v2 = p3 - p1;
            Eigen::Vector3f normal = v1.cross(v2);
            
            // Skip if the normal is zero (points are collinear)
            if (normal.norm() < 1e-6) {
                continue;
            }

            normal.normalize();

            // Compute plane equation: ax + by + cz + d = 0
            float d = -normal.dot(p1);
            Eigen::Vector4f plane(normal[0], normal[1], normal[2], d);

            // Find inliers (points close to the plane)
            std::vector<int> inlier_indices;
            for (size_t i = 0; i < points.size(); ++i) {
                if (pointToPlaneDistance(points[i], plane) < distance_threshold) {
                    inlier_indices.push_back(i);
                }
            }

            // Update the best plane if the current one has more inliers
            if (inlier_indices.size() > max_inliers) {
                max_inliers = inlier_indices.size();
                best_inlier_indices = inlier_indices;
                best_plane = plane;
            }
        }

        // Create a new matrix to store the non-ground points (points that are NOT inliers)
        std::vector<bool> is_inlier(points.size(), false);
        for (int idx : best_inlier_indices) {
            is_inlier[idx] = true;
        }

        // Count non-ground points
        int non_ground_count = points.size() - best_inlier_indices.size();
        
        // Create matrix for non-ground points
        Eigen::MatrixXf filtered_points(non_ground_count, 3);
        
        // Fill the matrix with non-ground points
        int row = 0;
        for (size_t i = 0; i < points.size(); ++i) {
            if (!is_inlier[i]) {
                filtered_points.row(row) = point_cloud.row(i);
                row++;
            }
        }

        return filtered_points;
    }

    struct GridCoordHash {
        std::size_t operator()(const Eigen::Vector3i& k) const {
            // Combine hashes for each component
            return std::hash<int>()(k[0]) ^ 
                (std::hash<int>()(k[1]) << 1) ^ 
                (std::hash<int>()(k[2]) << 2);
        }
    };

    // Custom equality operator for grid coordinates
    struct GridCoordEqual {
        bool operator()(const Eigen::Vector3i& lhs, const Eigen::Vector3i& rhs) const {
            return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
        }
    };

    Eigen::MatrixXf downsampleCloudEigen(const Eigen::MatrixXf& point_cloud, float leaf_size) {
        if (point_cloud.rows() == 0 || point_cloud.cols() < 3) {
            return point_cloud;
        }

        // Hash map to store voxel grid cells
        std::unordered_map<Eigen::Vector3i, std::vector<int>, GridCoordHash, GridCoordEqual> grid_map;

        // Find min and max for each dimension
        Eigen::Vector3f min_pt = point_cloud.colwise().minCoeff();
        Eigen::Vector3f max_pt = point_cloud.colwise().maxCoeff();

        // For each point, compute its grid coordinate and add to hash map
        for (int i = 0; i < point_cloud.rows(); ++i) {
            Eigen::Vector3f pt = point_cloud.row(i).head<3>();
            
            // Compute grid coordinates
            Eigen::Vector3i grid_coord;
            for (int j = 0; j < 3; ++j) {
                grid_coord[j] = static_cast<int>((pt[j] - min_pt[j]) / leaf_size);
            }

            // Add point index to corresponding voxel
            grid_map[grid_coord].push_back(i);
        }

        // Prepare output matrix
        Eigen::MatrixXf downsampled_cloud(grid_map.size(), point_cloud.cols());
        int row = 0;

        // For each voxel, compute centroid of points
        for (const auto& cell : grid_map) {
            const std::vector<int>& point_indices = cell.second;
            
            // Compute centroid for this voxel
            Eigen::VectorXf centroid = Eigen::VectorXf::Zero(point_cloud.cols());
            for (int idx : point_indices) {
                centroid += point_cloud.row(idx);
            }
            centroid /= point_indices.size();

            // Add centroid to output cloud
            downsampled_cloud.row(row) = centroid;
            row++;
        }

        return downsampled_cloud;
    }


    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr eigenToPCL(const Eigen::MatrixXf& eigen_mat) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
        cloud->resize(eigen_mat.rows());
        for (int i = 0; i < eigen_mat.rows(); ++i) {
            cloud->points[i].x = eigen_mat(i, 0);
            cloud->points[i].y = eigen_mat(i, 1);
            cloud->points[i].z = eigen_mat(i, 2);
        }
        return cloud;
    }

    Eigen::MatrixXf removeNaNRows(const Eigen::MatrixXf& input) {
        std::vector<int> validIndices;

        // Collect indices of rows that do NOT contain NaNs
        for (int i = 0; i < input.rows(); ++i) {
            if (!input.row(i).hasNaN()) {
                validIndices.push_back(i);
            }
        }

        // Create a new matrix with only valid rows
        Eigen::MatrixXf filtered(validIndices.size(), input.cols());
        for (size_t i = 0; i < validIndices.size(); ++i) {
            filtered.row(i) = input.row(validIndices[i]);
        }

        return filtered;
    }


private:
    ros::Subscriber here_are_the_clouds_sub_;
    ros::Publisher loop_closure_constraint_pub_;
    ros::Publisher debug_cloud1_pub_;
    ros::Publisher debug_cloud2_pub_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "loop_closer_node");
    ros::NodeHandle nh;
    LoopCloserNode lc_node(nh);

    ros::spin();
    // // Try using a multi-threaded spinner instead
    // ros::MultiThreadedSpinner spinner(2);  // Use 2 threads
    // spinner.spin();

    return 0;
}

