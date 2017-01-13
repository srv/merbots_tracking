#ifndef _TARGET_TRACKER_H
#define _TARGET_TRACKER_H

#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include <merbots_tracking/util/Params.h>
#include <merbots_tracking/util/PHOG.h>
#include <merbots_tracking/util/SharedData.h>
#include <merbots_tracking/wrappers/VisualTracker.h>
#include <merbots_tracking/wrappers/StruckVisualTracker.h>
#include <merbots_tracking/wrappers/KCFVisualTracker.h>

class TargetTracker
{
public:
    TargetTracker(const ros::NodeHandle& nodeh, Params* params, SharedData* shdata);
    ~TargetTracker();

    void run();

    void reset();
    void setTarget();

private:
    ros::NodeHandle nh;

    // Parameters
    Params* p;

    // Shared Data
    SharedData* sdata;

    // Internal parameters
    VisualTracker* vtracker;
    bool first_image;
    PHOG phog;
    cv::Mat target_descr;
};

#endif // _TARGET_TRACKER_H
