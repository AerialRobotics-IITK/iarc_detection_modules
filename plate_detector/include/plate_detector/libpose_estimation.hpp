#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <util_msgs/centre.h>

namespace iarc2020::plate_pose_estimation {

class PlatePoseEstimation {
    public:
    PlatePoseEstimation();

    void getDistance(float& dist);
    Eigen::Vector3d getGlobCoord() { return glob_coord_; };

    void setCamToQaud();
    void setCamMatrix();
    void setImgVec(float& x, float& y);
    void setQuaternion(nav_msgs::Odometry odom);

    void CamToQuad();
    void QuadToGlob(nav_msgs::Odometry odom);

    private:
    Eigen::Matrix3d scale_up_;
    Eigen::Matrix3d cam_matrix_;
    Eigen::Matrix3d cam_to_quad_;
    Eigen::Matrix3d quad_to_glob_;

    Eigen::Vector3d img_vec_;
    Eigen::Vector3d t_cam_;
    Eigen::Vector3d quad_coord_;
    Eigen::Vector3d glob_coord_;
};

}  // namespace iarc2020::plate_pose_estimation
