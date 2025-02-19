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

        ros::Rate rate(50);

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
            // pc1_matrix = removeNaNRows(pc1_matrix);
            // pc2_matrix = removeNaNRows(pc2_matrix);
        }

        cout << "1: " << pc1_matrix.size() << " 2: " << pc2_matrix.size() << endl;

        if (pc1_matrix.size() > 1000 && pc2_matrix.size() > 1000){
            // // RUN ICET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            // int run_length = 12;
            // int numBinsPhi = 24;
            // int numBinsTheta = 65; 
            // int n = 25; //min points per voxel
            // float thresh = 0.5;  // Jump threshold for beginning and ending radial clusters
            // float buff = 0.5;    //buffer to add to inner and outer cluster range (helps attract nearby distributions)
            // //seed initial estimate
            // Eigen::VectorXf X0(6);
            // // X0 << 0., 0., 0., 0., 0., 0.; 
            // //TODO: confirm this is correct!
            // X0 << -1*msg->X0[0], -1*msg->X0[1], -1*msg->X0[2], msg->X0[3], msg->X0[4], msg->X0[5]; 
            // cout << "X0: " << X0 << endl;
            // ICET it(pc1_matrix, pc2_matrix, run_length, X0, numBinsPhi, numBinsTheta, n, thresh, buff);
            // Eigen::VectorXf X = it.X;
            // cout << "soln: " << endl << X << endl;

            //RUN ICP instead (more robust with perspective shift) ~~~~~~~~~~~
            // Initialize ICP
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            // //use entire PC
            // icp.setInputSource(pcl_cloud1);
            // icp.setInputTarget(pcl_cloud2);
            //remove points too close to origin and downsample
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source = eigenToPCL(pc1_matrix);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target = eigenToPCL(pc2_matrix);
            removeGroundPlane(cloud_source);
            removeGroundPlane(cloud_target);
            float voxel_size_coarse = 0.5;  // 0.5 best so far...
            float voxel_size_fine = 0.2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_coarse = downsampleCloud(cloud_source, voxel_size_coarse);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_coarse = downsampleCloud(cloud_target, voxel_size_coarse);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_fine = downsampleCloud(cloud_source, voxel_size_fine);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_fine = downsampleCloud(cloud_target, voxel_size_fine);
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
                aligned_cloud->clear();
                icp.align(*aligned_cloud, final_transform);

                // Check if ICP diverged
                if (!icp.hasConverged()) {
                    X << 0., 0., 0., 0., 0., 0.; 
                    std::cout << "ICP did not converge!" << std::endl;
                    break;
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

            //publish as loop closure msg (need to send back to pose graph node)
            woodhouse::LoopClosed loop_closure_msg;

            // // //one way distances
            // double alignment_fitness = icp.getFitnessScore(); 
            //Bidirectional distances
            double forward_fitness = icp.getFitnessScore(2.0);
            icp.setInputSource(cloud_target_fine);
            icp.setInputTarget(aligned_cloud);
            double reverse_fitness = icp.getFitnessScore(2.0);
            double alignment_fitness = (forward_fitness + reverse_fitness)/2;

            double alignment_thresh = 0.50;
            if (alignment_fitness > alignment_thresh){
                loop_closure_msg.failed_to_converge = true;
            } else{
                loop_closure_msg.failed_to_converge = false;
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
            loop_closure_constraint_pub_.publish(loop_closure_msg);
        }        


    }

    void removeGroundPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::ExtractIndices<pcl::PointXYZ> extract;

        // Configure RANSAC
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.15);  // Adjust for your dataset (meters)
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            std::cerr << "No ground plane found!" << std::endl;
            return;
        }

        // Remove the ground plane
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);  // Keep only outliers (non-ground points)
        extract.filter(*cloud);

        // std::cout << "Ground plane removed. Remaining points: " << cloud->size() << std::endl;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, float leaf_size) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(input_cloud);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);  // Set voxel grid size
        voxel_filter.filter(*filtered_cloud);

        return filtered_cloud;
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
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "loop_closer_node");
    // ros::init(argc, argv, "loop_closer_node", ros::init_options::AnonymousName); //for debug-- allow creation of multiple of this node
    ros::NodeHandle nh;
    LoopCloserNode lc_node(nh);
    ros::spin();
    return 0;
}

