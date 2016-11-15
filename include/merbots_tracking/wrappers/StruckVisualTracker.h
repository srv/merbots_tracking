#ifndef STRUCKVTRACKER_H
#define STRUCKVTRACKER_H

#include <sstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <merbots_tracking/wrappers/VisualTracker.h>
#include <struck/src/Tracker.h>
#include <struck/src/Config.h>

class StruckVisualTracker : public VisualTracker
{
public:
    StruckVisualTracker();
    ~StruckVisualTracker();

    void configure(const std::string &config_file);
    void initialize(const cv::Mat &frame, const cv::Rect bbox);
    cv::Rect track(const cv::Mat &frame);

private:
    TrackerStruck* _tracker;
    Config* _config;
};

#endif // STRUCKVTRACKER_H
