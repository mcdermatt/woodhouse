#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>  // Add this line for tf2_ros::Buffer
#include <tf2/LinearMath/Quaternion.h>
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
#include <map>
#include "icet.h"
#include "utils.h"

using namespace std;
using namespace Eigen;

// Subscribes to "here are the clouds" topic and runs ICET point cloud registration

// publishes a 6DOF transform relatinig the two provided point clouds to one another 

// TODO-- should we also include an initial estimate to seed these transforms?
// TODO-- add flag for ICET solution digerging--- don't add LC constraint to graph if so   

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
            int run_length = 12;
            int numBinsPhi = 24;
            int numBinsTheta = 65; 
            int n = 25; //min points per voxel
            float thresh = 0.5;  // Jump threshold for beginning and ending radial clusters
            float buff = 0.5;    //buffer to add to inner and outer cluster range (helps attract nearby distributions)
            //seed initial estimate
            Eigen::VectorXf X0(6);
            // X0 << 0., 0., 0., 0., 0., 0.; 
            //TODO: confirm this is correct!
            X0 << -1*msg->X0[0], -1*msg->X0[1], -1*msg->X0[2], msg->X0[3], msg->X0[4], msg->X0[5]; 
            cout << "X0: " << X0 << endl;
            ICET it(pc1_matrix, pc2_matrix, run_length, X0, numBinsPhi, numBinsTheta, n, thresh, buff);
            Eigen::VectorXf X = it.X;
            cout << "soln: " << endl << X << endl;
            //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            //publish as loop closure msg (need to send back to pose graph node)
            woodhouse::LoopClosed loop_closure_msg;
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

