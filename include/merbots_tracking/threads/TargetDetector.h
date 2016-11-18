#ifndef _TARGET_DETECTOR_H
#define _TARGET_DETECTOR_H

#include <opencv2/calib3d.hpp>

#include <merbots_tracking/util/Params.h>
#include <merbots_tracking/util/SharedData.h>

class TargetDetector
{
public:
    TargetDetector(const ros::NodeHandle& nodeh, Params* params, SharedData* shdata);
    ~TargetDetector();

    void run();

private:
    ros::NodeHandle nh;

    // Parameters
    Params* p;

    // Shared Data
    SharedData* sdata;

    // Internal parameters
    cv::Mat curr_target;
    std::vector<cv::KeyPoint> obj_kps;
    cv::Mat obj_descs;
    cv::flann::Index* obj_ind;
    std::vector<cv::Point2f> obj_corners;
};

#endif // _TARGET_DETECTOR_H
