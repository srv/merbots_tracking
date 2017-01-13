#ifndef _TARGET_DETECTOR_H
#define _TARGET_DETECTOR_H

#include <opencv2/highgui.hpp>
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
};

#endif // _TARGET_DETECTOR_H
