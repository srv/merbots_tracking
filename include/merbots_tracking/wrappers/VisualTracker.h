#ifndef VISUALTRACKER_H
#define VISUALTRACKER_H

#include <opencv2/core.hpp>

class VisualTracker
{
public:
    VisualTracker()
    {}

    virtual void configure(const std::string& config_file) = 0;
    virtual void initialize(const cv::Mat& frame, const cv::Rect bbox) = 0;
    virtual cv::Rect track(const cv::Mat& frame) = 0;
};

#endif // VISUALTRACKER_H
