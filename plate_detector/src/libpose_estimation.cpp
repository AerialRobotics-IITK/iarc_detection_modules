#include <plate_detector/libpose_estimation.hpp>

namespace iarc2020::plate_pose_estimation {

PlatePoseEstimation::PlatePoseEstimation() {
    // clang-format off
    cam_matrix_ << 320.25492609007654, 0, 320.5,
                   0, 320.25492609007654, 240.5,
                   0, 0, 1;

    cam_to_quad_ << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    // clang-format on

    img_vec_ = Eigen::Vector3d(0, 0, 1);
    t_cam_ = Eigen::Vector3d(0, 0, 0.02);
}

void PlatePoseEstimation::getDistance(const float& dist) {
    // clang-format off
    scale_up_ << dist, 0, 0,
                 0, dist, 0,
                 0, 0, dist;
    // clang-format on
}

void PlatePoseEstimation::setImgVec(const float& x, const float& y) {
    img_vec_(0) = x;
    img_vec_(1) = y;
}

void PlatePoseEstimation::setQuaternion(const nav_msgs::Odometry& odom) {
    tf::Quaternion quat =
        tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
    Eigen::Quaterniond eigen_quat = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
    quad_to_glob_ = eigen_quat.normalized().toRotationMatrix();
}

void PlatePoseEstimation::CamToQuad() {
    ROS_INFO_STREAM(scale_up_(0, 0) << "\n");
    Eigen::Matrix3d inv_cam_matrix = cam_matrix_.inverse();
    quad_coord_ = cam_to_quad_ * scale_up_ * inv_cam_matrix * img_vec_ + t_cam_;
}

void PlatePoseEstimation::QuadToGlob(const nav_msgs::Odometry& odom) {
    glob_coord_ = quad_to_glob_ * quad_coord_;

    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
}

}  // namespace iarc2020::plate_pose_estimation
