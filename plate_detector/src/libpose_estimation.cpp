#include <plate_detector/libpose_estimation.hpp>

namespace iarc2020::plate_pose_estimation {

PlatePoseEstimation::PlatePoseEstimation() {
    // TODO: obtain camera parameters from topic
    cam_matrix_(0, 0) = 320.25492609007654;
    cam_matrix_(0, 1) = 0;
    cam_matrix_(0, 2) = 320.5;
    cam_matrix_(1, 0) = 0;
    cam_matrix_(1, 1) = 320.25492609007654;
    cam_matrix_(1, 2) = 240.5;
    cam_matrix_(2, 0) = 0;
    cam_matrix_(2, 1) = 0;
    cam_matrix_(2, 2) = 1.0;

    // TODO: from params
    cam_to_quad_(0, 0) = 0;
    cam_to_quad_(0, 1) = 0;
    cam_to_quad_(0, 2) = 1;
    cam_to_quad_(1, 0) = -1;
    cam_to_quad_(1, 1) = 0;
    cam_to_quad_(1, 2) = 0;
    cam_to_quad_(2, 0) = 0;
    cam_to_quad_(2, 1) = -1;
    cam_to_quad_(2, 2) = 0;

    img_vec_(0) = 0;
    img_vec_(1) = 0;
    img_vec_(2) = 1;

    // TODO: params
    t_cam_(0) = 0.0;
    t_cam_(1) = 0.0;
    t_cam_(2) = 0.02;
}
void PlatePoseEstimation::getDistance(float& dist) {
    for (int i = 0; i < 3; i += 1) {
        for (int j = 0; j < 3; j += 1) {
            if (i == j) {
                scale_up_(i, j) = dist;
            } else {
                scale_up_(i, j) = 0;
            }
        }
    }
}

void PlatePoseEstimation::setImgVec(float& x, float& y) {
    img_vec_(0) = x;
    img_vec_(1) = y;
}

void PlatePoseEstimation::setQuaternion(nav_msgs::Odometry odom) {
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

void PlatePoseEstimation::QuadToGlob(nav_msgs::Odometry odom) {
    glob_coord_ = quad_to_glob_ * quad_coord_;

    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
}

}  // namespace iarc2020::plate_pose_estimation
