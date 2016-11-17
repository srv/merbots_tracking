#include <merbots_tracking/util/SharedData.h>

SharedData* SharedData::_instance = 0;

/**
 * @brief Method for getting access to the singleton.
 * @return A reference to the unique object of the class.
 */
SharedData* SharedData::getInstance()
{
    if (_instance == 0)
    {
        _instance = new SharedData;
    }
    return _instance;
}

void SharedData::setROI(const int& x, const int& y, const int& width, const int& heigh)
{
    boost::mutex::scoped_lock lock(mutex_roi);
    roi.x = x;
    roi.y = y;
    roi.width = width;
    roi.height = heigh;
}

void SharedData::setROI(const cv::Rect& bbox)
{
    boost::mutex::scoped_lock lock(mutex_roi);
    roi = bbox;
}

cv::Rect SharedData::getROI()
{
    boost::mutex::scoped_lock lock(mutex_roi);
    return roi;
}

void SharedData::setStatus(const Status& curr_status)
{
    boost::mutex::scoped_lock lock(mutex_status);
    status = curr_status;
}

Status SharedData::getStatus()
{
    boost::mutex::scoped_lock lock(mutex_status);
    return status;
}

void SharedData::setTarget(const cv::Mat &image)
{
    boost::mutex::scoped_lock lock(mutex_target);
    image.copyTo(target);
}

cv::Mat SharedData::getTarget()
{
    boost::mutex::scoped_lock lock(mutex_target);
    return target;
}

void SharedData::setCurrentImage(const cv::Mat &image)
{
    boost::mutex::scoped_lock lock(mutex_cimage);
    image.copyTo(curr_image);
}

cv::Mat SharedData::getCurrentImage()
{
    boost::mutex::scoped_lock lock(mutex_cimage);
    return curr_image;
}

void SharedData::copyCurrentImage(cv::Mat &image)
{
    boost::mutex::scoped_lock lock(mutex_cimage);
    curr_image.copyTo(image);
}

bool SharedData::existsImage()
{
    boost::mutex::scoped_lock lock(mutex_cimage);
    return !curr_image.empty();
}
