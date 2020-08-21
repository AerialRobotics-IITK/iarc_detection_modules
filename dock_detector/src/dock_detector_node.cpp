#include <dock_detector/dock_detector.hpp>

using namespace ariitk::dock_detection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "dock_detector_node");
    ros::NodeHandle nh;

    DockDetectorNode detect;

    detect.init(nh);

    ros::Rate loop_rate(20);

    while (ros::ok()) {
        detect.run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
