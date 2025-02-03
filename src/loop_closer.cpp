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
#include <map>
#include "icet.h"
#include "utils.h"

using namespace std;
using namespace Eigen;

// Subscribes to "here are the clouds" topic and runs ICET point cloud registration

// publishes a 6DOF transform relatinig the two provided point clouds to one another 

//TODO-- should we also include an initial estimate to seed these transforms?

class LoopCloserNode {
public:
    LoopCloserNode(ros::NodeHandle& nh){
        // Set up ROS subscribers and publishers
        here_are_the_clouds_sub_ = nh.subscribe("/here_are_the_clouds", 10, &LoopCloserNode::hereAreTheCloudsCallback, this);

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
        float minD = 0.2;
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
            // RUN ICET ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            int run_length = 7;
            int numBinsPhi = 24;
            int numBinsTheta = 75; 
            //seed initial estimate -- TODO-- update msg to add seed TF
            Eigen::VectorXf X0(6);
            X0 << 0., 0., 0., 0., 0., 0.; 
            // X0 << X[0], X[1], X[2], X[3], X[4], X[5]; 
            ICET it(pc1_matrix, pc2_matrix, run_length, X0, numBinsPhi, numBinsTheta);
            Eigen::VectorXf X = it.X;
            cout << "soln: " << endl << X << endl;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

            // std::cout << pc1_matrix.block(0, 0, 10, 3) << std::endl;
            // std::cout << pc2_matrix.block(0, 0, 10, 3) << std::endl;

            //TODO: send resulting constraint as msg to pose graph node
        }        

    }

    Eigen::MatrixXf convertPCLtoEigen(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud) {
        Eigen::MatrixXf eigen_matrix(pcl_cloud->size(), 3);
        for (size_t i = 0; i < pcl_cloud->size(); ++i) {
            eigen_matrix.row(i) << pcl_cloud->points[i].x, pcl_cloud->points[i].y, pcl_cloud->points[i].z;
        }
        return eigen_matrix;
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
    // ros::Publisher here_are_the_clouds_pub_;

};

int main(int argc, char** argv) {
    // ros::init(argc, argv, "loop_closer_node");
    ros::init(argc, argv, "loop_closer_node", ros::init_options::AnonymousName); //for debug-- allow creation of multiple of this node
    ros::NodeHandle nh;
    LoopCloserNode lc_node(nh);
    ros::spin();
    return 0;
}

