#pragma once

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

namespace iarc2020::pose_estimation {

class PoseEstimator {
    public:
    void init();

    void getDistance(const float& dist);
    Eigen::Vector3d getGlobCoord() { return glob_coord_; };

    void setCamToQuadMatrix(const std::vector<double>& mat) { cam_to_quad_ = Eigen::Matrix3d(mat.data()); };
    void setCamMatrix(const std::vector<double>& mat) {
        cam_matrix_ = Eigen::Matrix3d(mat.data());
        inv_cam_matrix_ = cam_matrix_.inverse();
    };
    void setTCamMatrix(const std::vector<double>& mat) { t_cam_ = Eigen::Vector3d(mat.data()); };
    void setVerbosity(const bool& flag) { verbose_ = flag; }

    void setImgVec(const float& x, const float& y);
    void setQuaternion(const nav_msgs::Odometry& odom);

    void CamToQuad();
    void QuadToGlob(const nav_msgs::Odometry& odom);

    private:
    Eigen::Matrix3d scale_up_;
    Eigen::Matrix3d cam_matrix_;
    Eigen::Matrix3d inv_cam_matrix_;
    Eigen::Matrix3d cam_to_quad_;
    Eigen::Matrix3d quad_to_glob_;

    Eigen::Vector3d img_vec_;
    Eigen::Vector3d t_cam_;
    Eigen::Vector3d quad_coord_;
    Eigen::Vector3d glob_coord_;

    bool verbose_;
};

}  // namespace iarc2020::pose_estimation
