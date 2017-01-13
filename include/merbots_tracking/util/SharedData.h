#ifndef _SHAREDDATA_H
#define _SHAREDDATA_H

#include <boost/thread.hpp>
#include <opencv2/core.hpp>
#include <ros/ros.h>

#include <merbots_tracking/util/Params.h>

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
    void setScenePoints(const std::vector<cv::Point2f>& pts);
    void getScenePoints(std::vector<cv::Point2f>& pts);
    void setStatus(const Status& curr_status);
    Status getStatus();
    void setTarget(const cv::Mat& image);
    cv::Mat getTarget();
    void searchKeypoints(const cv::Mat& descs, cv::Mat& results, cv::Mat& dists, int knn = 2);
    cv::Point2f getKeypoint(int index);
    void copyCurrentTarget(cv::Mat& image);
    bool existsTarget();
    void getTargetPoints(std::vector<cv::Point2f>& pts);
    void setCurrentImage(const cv::Mat& image);
    cv::Mat getCurrentImage();
    void copyCurrentImage(cv::Mat& image);
    bool existsImage();

    boost::mutex mutex_upd_target;

protected:
    // Protected constructor. Singleton class.
    SharedData() :
        status(DETECTION),
        target_ind(0)
    {
        p = Params::getInstance();
    }

    ~SharedData()
    {
        delete _instance;
    }

    SharedData(const SharedData &);
    SharedData& operator=(const SharedData &);

    boost::mutex mutex_roi;
    cv::Rect roi;

    boost::mutex mutex_points;
    std::vector<cv::Point2f> points;

    boost::condition_variable status_condvar;
    boost::mutex mutex_status;
    Status status;

    boost::mutex mutex_target;
    cv::Mat target;
    std::vector<cv::KeyPoint> target_kps;
    cv::Mat target_descs;
    cv::flann::Index* target_ind;
    std::vector<cv::Point2f> target_pts;

    boost::mutex mutex_cimage;
    cv::Mat curr_image;

    // Parameters
    Params* p;

private:
    // Single instance.
    static SharedData* _instance;
};

#endif /* _SHAREDDATA_H */
