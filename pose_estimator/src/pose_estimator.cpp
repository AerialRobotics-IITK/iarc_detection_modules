#include <pose_estimator/pose_estimator.hpp>

namespace ariitk::pose_estimation {

void PoseEstimatorNode::init(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
    centre_coord_sub_ = nh.subscribe("centre_coord", 10, &PoseEstimatorNode::centreCallback, this);
    odom_sub_ = nh.subscribe("odom", 10, &PoseEstimatorNode::odomCallback, this);
    corners_sub_ = nh.subscribe("corners", 10, &PoseEstimatorNode::cornersCallback, this);

    glob_coord_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("estimated_coord", 10);
    front_coord_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("front_coord", 10);
    plate_front_vec_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("plate_front_vec", 10);
    yaw_correction_pub_ = nh_private.advertise<detector_msgs::GlobalCoord>("yaw_correction", 10);

    pose_est_.init();

    std::vector<double> temp_list;
    nh_private.getParam("camera_matrix", temp_list);
    pose_est_.setCamMatrix(temp_list);

    temp_list.clear();
    nh_private.getParam("cam_to_quad_rot", temp_list);
    pose_est_.setCamToQuadMatrix(temp_list);

    temp_list.clear();
    nh_private.getParam("t_cam", temp_list);
    pose_est_.setTCamMatrix(temp_list);

    bool verbose_flag = true;
    nh_private.param("verbose", verbose_flag, true);
    pose_est_.setVerbosity(verbose_flag);

    nh_private.getParam("image_height", image_height_);
    nh_private.getParam("image_width", image_width_);
}

void PoseEstimatorNode::run() {
    calculateScalingFactor();

    straight_vec_ = calculateGlobCoord(image_width_ / 2, image_height_ / 2, dist_);
    straight_vec_(0) -= odom_.pose.pose.position.x;  //* Converting straight_vec from position vec to direction vec
    straight_vec_(1) -= odom_.pose.pose.position.y;
    straight_vec_(2) -= odom_.pose.pose.position.z;
    front_coord_.x = straight_vec_(0);
    front_coord_.y = straight_vec_(1);
    front_coord_.z = straight_vec_(2);

    front_coord_pub_.publish(front_coord_);

    if ((centre_coord_.x == -1) || (centre_coord_.y == -1)) {
        glob_coord_pub_.publish(global_coord_);
        return;
    }

    glob_coord_ = calculateGlobCoord(centre_coord_.x, centre_coord_.y, dist_);
    global_coord_.x = glob_coord_(0);
    global_coord_.y = glob_coord_(1);
    global_coord_.z = glob_coord_(2);
    glob_coord_pub_.publish(global_coord_);

    plate_front_vec_temp_ = calculatePlateFrontVec();
    plate_front_vec_.x = plate_front_vec_temp_(0);
    plate_front_vec_.y = plate_front_vec_temp_(1);
    plate_front_vec_.z = plate_front_vec_temp_(2);
    plate_front_vec_pub_.publish(plate_front_vec_);

    publishCorrectionAngles();
}

Eigen::Vector3d PoseEstimatorNode::calculateGlobCoord(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist_);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad();
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlob(odom_);
    return pose_est_.getGlobCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculateQuadCoord(const double& img_x, const double& img_y, const double& dist) {
    pose_est_.getDistance(dist_);
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuad();
    return pose_est_.getQuadCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculateQuadCoordForDist(const double& img_x, const double& img_y) {
    pose_est_.setImgVec(img_x, img_y);
    pose_est_.CamToQuadForDist();
    return pose_est_.getQuadCoord();
}

Eigen::Vector3d PoseEstimatorNode::calculatePlateFrontVec() {
    c1_quad_coord_ = calculateQuadCoord(corners_.c1_x, corners_.c1_y, lamdas_[0]);
    c2_quad_coord_ = calculateQuadCoord(corners_.c2_x, corners_.c2_y, lamdas_[1]);
    c3_quad_coord_ = calculateQuadCoord(corners_.c3_x, corners_.c3_y, lamdas_[2]);
    c4_quad_coord_ = calculateQuadCoord(corners_.c4_x, corners_.c4_y, lamdas_[3]);

    Eigen::Vector3d cross_p;
    cross_p = (c2_quad_coord_ - c4_quad_coord_).cross(c1_quad_coord_ - c3_quad_coord_);
    pose_est_.setQuaternion(odom_);
    pose_est_.QuadToGlobPlateFrontVec(odom_, cross_p);
    return pose_est_.getPlateFrontVec();
}

void PoseEstimatorNode::calculateScalingFactor() {
    c1_quad_coord_ = calculateQuadCoordForDist(corners_.c1_x, corners_.c1_y);
    c2_quad_coord_ = calculateQuadCoordForDist(corners_.c2_x, corners_.c2_y);
    c3_quad_coord_ = calculateQuadCoordForDist(corners_.c3_x, corners_.c3_y);
    c4_quad_coord_ = calculateQuadCoordForDist(corners_.c4_x, corners_.c4_y);

    // Eigen::Vector3d cross_p;
    // cross_p = (c1_quad_coord_ - c2_quad_coord_).cross(c3_quad_coord_ - c2_quad_coord_);
    // area_ = sqrt(cross_p.dot(cross_p));
    // dist_ = sqrt(actual_area_ / area_);
    // std::cout << "[ " << dist_ << " ]" << std::endl << std::endl;
    // NR<Eigen::Vector4d, Eigen::Matrix4d, PoseEstimatorNode> solve(&PoseEstimatorNode::l_function, &PoseEstimatorNode::l_derivative_inv, this);
    // solve.setConvergence(&PoseEstimatorNode::l_convergence);
    Eigen::Vector4d initial = {centre_coord_.d, centre_coord_.d, centre_coord_.d, centre_coord_.d};
    while (!l_convergence(l_function(initial))) initial = (initial - (l_derivative_inv(initial) * l_function(initial)));

    // Eigen::Vector4d roots = solve.root(guess);
    for (int i = 0; i < 4; i++) lamdas_[i] = initial(i);

}

void PoseEstimatorNode::publishCorrectionAngles() {
    //* Calculating Correction Angles(to be shifted to traj pkg after completion)
    float mod_v1 = 0, mod_v2 = 0;
    Eigen::Vector3d temp, v1, v2;

    v1 = Eigen::Vector3d(0, glob_coord_(1) - odom_.pose.pose.position.y, glob_coord_(2) - odom_.pose.pose.position.z);
    v2 = Eigen::Vector3d(0, plate_front_vec_temp_(1), plate_front_vec_temp_(2));
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    yaw_correction_.x = asin(((v1(1) * v2(2)) - (v1(2) * v2(1))) / (mod_v1 * mod_v2));

    v1 = Eigen::Vector3d(glob_coord_(0) - odom_.pose.pose.position.x, 0, glob_coord_(2) - odom_.pose.pose.position.z);
    v2 = Eigen::Vector3d(plate_front_vec_temp_(0), 0, plate_front_vec_temp_(2));
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    yaw_correction_.y = asin(((v1(0) * v2(2)) - (v1(2) * v2(0))) / (mod_v1 * mod_v2));

    v1 = Eigen::Vector3d(glob_coord_(0) - odom_.pose.pose.position.x, glob_coord_(1) - odom_.pose.pose.position.y, 0);
    v2 = Eigen::Vector3d(plate_front_vec_temp_(0), plate_front_vec_temp_(1), 0);
    mod_v1 = sqrt(v1.dot(v1));
    mod_v2 = sqrt(v2.dot(v2));
    yaw_correction_.z = asin(((v1(0) * v2(1)) - (v1(1) * v2(0))) / (mod_v1 * mod_v2));

    yaw_correction_pub_.publish(yaw_correction_);

    // std::cout << "[ " << yaw_correction_.x << " " << yaw_correction_.y << " " << yaw_correction_.z << " ]" << std::endl << std::endl;
}

void PoseEstimatorNode::centreCallback(const detector_msgs::Centre& msg) {
    centre_coord_ = msg;
}

void PoseEstimatorNode::odomCallback(const nav_msgs::Odometry& msg) {
    odom_ = msg;
}

void PoseEstimatorNode::cornersCallback(const detector_msgs::Corners& msg) {
    corners_ = msg;
}

Eigen::Vector4d PoseEstimatorNode::l_function(Eigen::Vector4d var) {
    Eigen::Vector4d result;
    Eigen::Vector3d c[4] = {c1_quad_coord_, c2_quad_coord_, c3_quad_coord_, c4_quad_coord_};

    // result(0) = lamda_one * lamda_two * (c1_quad_coord_.dot(c2_quad_coord_)) - lamda_one * lamda_three * (c1_quad_coord_.dot(c3_quad_coord_)) -
    //             lamda_two * lamda_two * (c2_quad_coord_.dot(c2_quad_coord_)) + lamda_three * lamda_two * (c2_quad_coord_.dot(c3_quad_coord_));

    // result(1) = lamda_two * lamda_three * (c2_quad_coord_.dot(c3_quad_coord_)) - lamda_two * lamda_four * (c2_quad_coord_.dot(c4_quad_coord_)) -
    //             lamda_three * lamda_three * (c3_quad_coord_.dot(c3_quad_coord_)) + lamda_four * lamda_three * (c4_quad_coord_.dot(c3_quad_coord_));

    // result(2) = lamda_three * lamda_four * (c3_quad_coord_.dot(c4_quad_coord_)) - lamda_three * lamda_one * (c3_quad_coord_.dot(c1_quad_coord_)) -
    //             lamda_four * lamda_four * (c4_quad_coord_.dot(c4_quad_coord_)) + lamda_one * lamda_four * (c1_quad_coord_.dot(c4_quad_coord_));

    // result(3) = lamda_four * lamda_one * (c4_quad_coord_.dot(c1_quad_coord_)) - lamda_four * lamda_two * (c4_quad_coord_.dot(c2_quad_coord_)) -
    //             lamda_one * lamda_one * (c1_quad_coord_.dot(c1_quad_coord_)) + lamda_two * lamda_one * (c2_quad_coord_.dot(c1_quad_coord_));

    for (int i = 0; i < 4; i++) {
        double lamda_one = var[i%4];
        double lamda_two = var[(i+1)%4];
        double lamda_three = var[(i+2)%4];

        Eigen::Vector3d c1 = c[i%4];
        Eigen::Vector3d c2 = c[(i+1)%4];
        Eigen::Vector3d c3 = c[(i+2)%4];


        result(i) = lamda_one * lamda_two * (c1.dot(c2)) - lamda_one * lamda_three * (c1.dot(c3)) -
                    lamda_two * lamda_two * (c2.dot(c2)) + lamda_three * lamda_two * (c2.dot(c3));
    }
    
    return result;
}

Eigen::Matrix4d PoseEstimatorNode::l_derivative_inv(Eigen::Vector4d var){
    Eigen::Matrix4d result;
    Eigen::Vector3d c[4] = {c1_quad_coord_, c2_quad_coord_, c3_quad_coord_, c4_quad_coord_};

    // double lamda_one = var[0];
    // double lamda_two = var[1];
    // double lamda_three = var[2];
    // double lamda_four = var[3];

    for (int i = 0; i < 4; i++) {
        double lamda_one = var[i%4];
        double lamda_two = var[(i+1)%4];
        double lamda_three = var[(i+2)%4];

        Eigen::Vector3d c1 = c[i%4];
        Eigen::Vector3d c2 = c[(i+1)%4];
        Eigen::Vector3d c3 = c[(i+2)%4];

        result(i, i%4) = lamda_two*(c1.dot(c2)) - lamda_three*(c1.dot(c3));
        result(i, (i+1)%4) = lamda_one*(c1.dot(c2)) - 2*lamda_two*(c2.dot(c2)) + lamda_three*(c2.dot(c3));
        result(i, (i+2)%4) = -lamda_one*(c1.dot(c2)) + lamda_two*(c2.dot(c3));
        result(i, (i+3)%4) = 0;
    }

    return result.inverse();
}

    bool PoseEstimatorNode::l_convergence(Eigen::Vector4d var) {
        return (var.norm() < 0.01);
    }
}  // namespace ariitk::pose_estimation
