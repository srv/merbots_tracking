#ifndef _PARAMS_H
#define _PARAMS_H

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <ros/ros.h>

/**
 * @brief Singleton class to store the parameters of the application.
 */
class Params
{
public:
    // Parameters //

    // Global parameters
    bool debug;

    // Target detection parameters
    cv::Ptr<cv::Feature2D> det_detector;
    cv::Ptr<cv::Feature2D> det_descriptor;
    double det_rerror;
    int det_inliers;
    std::string det_target_from_file;
    bool use_camera;

    // Visual tracking parameters
    std::string track_tracker;

    // Public functions
    static Params* getInstance();
    void readParams(const ros::NodeHandle& nh);

protected:
    // Protected constructor. Singleton class.
    Params() :
        debug(true),
        det_rerror(3.0),
        det_inliers(25),
        det_target_from_file(""),
        use_camera(true),
        track_tracker("kcf")
    {        
    }

    ~Params()
    {
        delete _instance;
    }

    Params(const Params &);
    Params& operator=(const Params &);

private:
    // Single instance.
    static Params* _instance;
};

#endif /* _PARAMS_H */
