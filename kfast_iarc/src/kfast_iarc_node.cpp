#include <kfast_iarc/KFAST.hpp>

using namespace ariitk::kfast;

int main(int argc, char** argv) {
    ros::init(argc, argv, "KFAST_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    kfast kf;

    kf.init(nh, nh_private);

    ros::Rate loopRate(10);

    while (ros::ok()) {
        ros::spinOnce();
        kf.run();
        loopRate.sleep();
    }
    return 0;
}