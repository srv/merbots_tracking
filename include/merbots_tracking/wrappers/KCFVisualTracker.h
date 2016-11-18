#ifndef KCFVTRACKER_H
#define KCFVTRACKER_H

#include <merbots_tracking/wrappers/VisualTracker.h>

#include <kcf/src/kcftracker.hpp>

class KCFVisualTracker : public VisualTracker
{
public:
    KCFVisualTracker();
    ~KCFVisualTracker();

    void configure(const std::string &config_file);
    void initialize(const cv::Mat &frame, const cv::Rect bbox);
    cv::Rect track(const cv::Mat &frame);

private:
    KCFTracker* _tracker;
};

#endif // KCFVTRACKER_H
