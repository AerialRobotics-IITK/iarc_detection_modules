#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <math.h>

#include <detector_msgs/Centre.h>
#include <detector_msgs/GlobalCoord.h>
#include <detector_msgs/Corners.h>
#include <pose_estimator/libpose_estimation.hpp>

namespace iarc2020::pose_estimation {

class PoseEstimatorNode {
  public:
    void init(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

  private:
    Eigen::Vector3d calculateGlobCoord(const double& img_x, const double& img_y, const double& dist);
    Eigen::Vector3d calculateQuadCoord(const double& img_x, const double& img_y, const double& dist);
    Eigen::Vector3d calculateQuadCoordForDist(const double& img_x, const double& img_y);
    Eigen::Vector3d calculatePlateFrontVec();
    void publishCorrectionAngles();
    void calculateScalingFactor();
    void centreCallback(const detector_msgs::Centre& msg);
    void odomCallback(const nav_msgs::Odometry& msg);
    void cornersCallback(const detector_msgs::Corners& msg);

    int image_height_;
    int image_width_;
    float area_;
    float actual_area_ = 0.095;
    double dist_;

    detector_msgs::Centre centre_coord_;
    detector_msgs::GlobalCoord global_coord_;
    detector_msgs::GlobalCoord front_coord_;
    detector_msgs::GlobalCoord plate_front_vec_;
    detector_msgs::Corners corners_;
    detector_msgs::GlobalCoord yaw_correction_;

    nav_msgs::Odometry odom_;
    Eigen::Vector3d glob_coord_;
    Eigen::Vector3d c1_quad_coord_;
    Eigen::Vector3d c2_quad_coord_;
    Eigen::Vector3d c3_quad_coord_;
    Eigen::Vector3d c4_quad_coord_;
    Eigen::Vector3d plate_front_vec_temp_;
    Eigen::Vector3d straight_vec_;

    ros::Subscriber centre_coord_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber corners_sub_;

    ros::Publisher glob_coord_pub_;
    ros::Publisher front_coord_pub_;
    ros::Publisher plate_front_vec_pub_;
    ros::Publisher yaw_correction_pub_;

    PoseEstimator pose_est_;
};

}  // namespace iarc2020::pose_estimation
