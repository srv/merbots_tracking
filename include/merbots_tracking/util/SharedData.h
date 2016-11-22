#ifndef _SHAREDDATA_H
#define _SHAREDDATA_H

#include <boost/thread.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

enum Status {DETECTION, TRACKING};

/**
 * @brief Singleton class to store shared structures of the application.
 */
class SharedData
{
public:
    // Public functions.
    static SharedData* getInstance();
    void setROI(const int& x, const int& y, const int& width, const int &heigh);
    void setROI(const cv::Rect& bbox);
    cv::Rect getROI();
    void setStatus(const Status& curr_status);
    Status getStatus();
    void setTarget(const cv::Mat& image);
    cv::Mat getTarget();
    void copyCurrentTarget(cv::Mat& image);
    bool existsTarget();
    void setCurrentImage(const cv::Mat& image);
    cv::Mat getCurrentImage();
    void copyCurrentImage(cv::Mat& image);
    bool existsImage();

    boost::mutex mutex_upd_target;

protected:
    // Protected constructor. Singleton class.
    SharedData() :
        status(DETECTION)
    {
    }

    ~SharedData()
    {
        delete _instance;
    }

    SharedData(const SharedData &);
    SharedData& operator=(const SharedData &);

    boost::mutex mutex_roi;
    cv::Rect roi;

    boost::condition_variable status_condvar;
    boost::mutex mutex_status;
    Status status;

    boost::mutex mutex_target;
    cv::Mat target;

    boost::mutex mutex_cimage;
    cv::Mat curr_image;

private:
    // Single instance.
    static SharedData* _instance;
};

#endif /* _SHAREDDATA_H */
