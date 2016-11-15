#include <merbots_tracking/wrappers/StruckVisualTracker.h>

StruckVisualTracker::StruckVisualTracker() :
    VisualTracker()
{
    std::string defaultconfig = ros::package::getPath("merbots_tracking") + "/trackers/struck/config.txt";
    _config = new Config(defaultconfig);

    std::stringstream ss;
    ss << *_config;
    ROS_INFO("%s", ss.str().c_str());

    _tracker = new TrackerStruck(*_config);
}

StruckVisualTracker::~StruckVisualTracker()
{
    delete _tracker;
    delete _config;
}

void StruckVisualTracker::configure(const std::string &config_file)
{
    if (_tracker) delete _tracker;
    if (_config) delete _config;

    _config = new Config(config_file);

    std::stringstream ss;
    ss << *_config;
    ROS_INFO("%s", ss.str().c_str());

    _tracker = new TrackerStruck(*_config);

}

void StruckVisualTracker::initialize(const cv::Mat &frame, const cv::Rect bbox)
{
    FloatRect r(bbox.x, bbox.y, bbox.width, bbox.height);
    _tracker->Initialise(frame, r);
}

cv::Rect StruckVisualTracker::track(const cv::Mat &frame)
{    
    _tracker->Track(frame);
    FloatRect r = _tracker->GetBB();

    return cv::Rect(r.XMin(), r.YMin(), r.Width(), r.Height());
}
