#include <feature_detection/text_detect.hpp>

using namespace ariitk::text_detect;

int main(int argc,char** argv)
{
    ros::init(argc,argv,"text_detect_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    text_detect detect;
    ROS_INFO("working bef init");
    detect.init(nh,nh_private);
    ROS_INFO("working after init");
    ros::Rate loopRate(2);

    while(ros::ok())
    {
        detect.run();
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}
