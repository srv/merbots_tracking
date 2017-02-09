#include <merbots_tracking/object_detector/ObjectDetector.h>

// Main function
int main(int argc, char** argv)
{
    // ROS
    ros::init(argc, argv, "det_and_track");
    ros::NodeHandle nh("~");

    ROS_INFO("MERBOTS detection and tracking");
    merbots_tracking::ObjectDetector objdet(nh);
    objdet.start();

    return 0;
}
