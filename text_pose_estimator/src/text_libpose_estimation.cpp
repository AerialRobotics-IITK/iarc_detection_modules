#include <text_pose_estimator/text_libpose_estimation.hpp>

namespace ariitk::text_pose_estimation {

void TextPoseEstimator::init() {
    // clang-format off
    cam_matrix_ << 415.21159208661874, 0, 424.5,
                   0, 415.21159208661874, 240.5,
                   0, 0, 1;

    cam_to_quad_ << 0, 0, 1,
                    -1, 0, 0,
                    0, -1, 0;
    // clang-format on

    img_vec_ = Eigen::Vector3d(0, 0, 1);
    t_cam_ = Eigen::Vector3d(0, 0, 0.02);
}

void TextPoseEstimator::getDistance(const float& dist) {
    // clang-format off
    scale_up_ << dist, 0, 0,
                 0, dist, 0,
                 0, 0, dist;
    // clang-format on
}

void TextPoseEstimator::setImgVec(const float& x, const float& y) {
    img_vec_(0) = x;
    img_vec_(1) = y;
}

void TextPoseEstimator::setQuaternion(const nav_msgs::Odometry& odom) {
    geometry_msgs::Quaternion odom_quat = odom.pose.pose.orientation;
    tf::Quaternion quat = tf::Quaternion(odom_quat.x, odom_quat.y, odom_quat.z, odom_quat.w);
    Eigen::Quaterniond eigen_quat = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
    quad_to_glob_ = eigen_quat.normalized().toRotationMatrix();
}

void TextPoseEstimator::CamToQuad() {
    quad_coord_ = cam_to_quad_ * scale_up_ * cam_matrix_.inverse() * img_vec_ + t_cam_;
}

void TextPoseEstimator::CamToQuadForDist() {
    quad_coord_ = cam_to_quad_ * cam_matrix_.inverse() * img_vec_ + t_cam_;
}

void TextPoseEstimator::QuadToGlob(const nav_msgs::Odometry& odom) {
    glob_coord_ = quad_to_glob_ * quad_coord_;

    glob_coord_(0) = glob_coord_(0) + odom.pose.pose.position.x;
    glob_coord_(1) = glob_coord_(1) + odom.pose.pose.position.y;
    glob_coord_(2) = glob_coord_(2) + odom.pose.pose.position.z;
}

void TextPoseEstimator::QuadToGlobPlateFrontVec(const nav_msgs::Odometry& odom, const Eigen::Vector3d cross_p) {
    plate_front_vec_ = quad_to_glob_ * cross_p;
}

Eigen::Vector3d TextPoseEstimator::getGlobCoord() {
    return glob_coord_;
}

Eigen::Vector3d TextPoseEstimator::getQuadCoord() {
    return quad_coord_;
}

Eigen::Vector3d TextPoseEstimator::getPlateFrontVec() {
    return plate_front_vec_;
}

void TextPoseEstimator::setCamToQuadMatrix(const std::vector<double>& mat) {
    cam_to_quad_ = Eigen::Matrix3d(mat.data()).transpose();
}

void TextPoseEstimator::setCamMatrix(const std::vector<double>& mat) {
    cam_matrix_ = Eigen::Matrix3d(mat.data()).transpose();
}

void TextPoseEstimator::setTCamMatrix(const std::vector<double>& mat) {
    t_cam_ = Eigen::Vector3d(mat.data());
}

void TextPoseEstimator::setVerbosity(const bool& flag) {
    verbose_ = flag;
}

}  // namespace ariitk::text_pose_estimation
