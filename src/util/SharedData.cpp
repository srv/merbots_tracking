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

void SharedData::setScenePoints(const std::vector<cv::Point2f>& pts)
{
    boost::mutex::scoped_lock lock(mutex_points);
    points.clear();
    points = pts;
}

void SharedData::getScenePoints(std::vector<cv::Point2f>& pts)
{
    boost::mutex::scoped_lock lock(mutex_points);
    pts.clear();
    pts = points;
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

    ROS_INFO("New target received");

    // Detecting and describing keypoints
    target_kps.clear();
    p->det_detector->detect(target, target_kps);
    ROS_INFO("Target: %i keypoints found", (int) target_kps.size());
    p->det_descriptor->compute(target, target_kps, target_descs);
    ROS_INFO("Target: %i descriptors found", (int) target_descs.rows);

    if (target_ind)
    {
        delete target_ind;
    }

    if (target_descs.type() == CV_8U)
    {
        // Binary descriptors detected (from ORB or BRIEF)

        // Create Flann LSH index
        target_ind = new cv::flann::Index(target_descs, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
    } else
    {
        // assume it is CV_32F
        // Create a Flann KDTree index
        target_ind = new cv::flann::Index(target_descs, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
    }

    // Storing the corners of the target
    target_pts.clear();
    target_pts.push_back(cv::Point(0, 0));
    target_pts.push_back(cv::Point(target.cols, 0));
    target_pts.push_back(cv::Point(0, target.rows));
    target_pts.push_back(cv::Point(target.cols, target.rows));

    ROS_INFO("Target correctly received in the detector thread");

    // cv::imshow("Target", curr_target);
    // cv::waitKey(0);
}

cv::Mat SharedData::getTarget()
{
    boost::mutex::scoped_lock lock(mutex_target);
    return target;
}

void SharedData::searchKeypoints(const cv::Mat& descs, cv::Mat& results, cv::Mat& dists, int knn)
{
    boost::mutex::scoped_lock lock(mutex_target);
    target_ind->knnSearch(descs, results, dists, knn);
}

cv::Point2f SharedData::getKeypoint(int index)
{
    boost::mutex::scoped_lock lock(mutex_target);
    return target_kps[index].pt;
}

void SharedData::copyCurrentTarget(cv::Mat& image)
{
    boost::mutex::scoped_lock lock(mutex_target);
    target.copyTo(image);
}

bool SharedData::existsTarget()
{
    boost::mutex::scoped_lock lock(mutex_target);
    return !target.empty();
}

void SharedData::getTargetPoints(std::vector<cv::Point2f>& pts)
{
    boost::mutex::scoped_lock lock(mutex_target);
    pts.clear();
    pts = target_pts;
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

void SharedData::setInliers(const int& inliers)
{
    boost::mutex::scoped_lock lock(mutex_inliers);
    last_inliers = inliers;
}

int SharedData::getInliers()
{
    boost::mutex::scoped_lock lock(mutex_inliers);
    return last_inliers;
}
