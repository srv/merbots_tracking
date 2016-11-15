#include <merbots_tracking/wrappers/KCFVisualTracker.h>

KCFVisualTracker::KCFVisualTracker() :
    VisualTracker()
{ 
    _tracker = new KCFTracker(true, false, true, false);
}

KCFVisualTracker::~KCFVisualTracker()
{
    delete _tracker;
}

void KCFVisualTracker::configure(const std::string &config_file)
{    
}

void KCFVisualTracker::initialize(const cv::Mat &frame, const cv::Rect bbox)
{
    _tracker->init(bbox, frame);
}

cv::Rect KCFVisualTracker::track(const cv::Mat &frame)
{
    return _tracker->update(frame);
}
