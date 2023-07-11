#include "Eigen/Eigen"
#include "math_tools.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>


void send_joint_positions(ros::Publisher publisher, Eigen::VectorXd q);

void computeAndSendTrajectory(Eigen::VectorXd q, Eigen::VectorXd qf, float t, int steps, ros::Publisher publisher);

bool compute_and_send_trajectory_2(
    pinocchio::Model model, 
    Eigen::Vector3d target_position, 
    Eigen::Vector3d target_orientation_rpy, 
    Eigen::VectorXd q0
);