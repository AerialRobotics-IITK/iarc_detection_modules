#include <plate_detector/plate_detector.hpp>

using namespace ariitk::plate_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "plate_detector_node");
    ros::NodeHandle nh;

    PlateDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
