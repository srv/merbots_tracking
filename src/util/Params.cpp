#include <merbots_tracking/util/Params.h>

Params* Params::_instance = 0;

/**
 * @brief Method for getting access to the singleton.
 * @return A reference to the unique object of the class.
 */
Params* Params::getInstance()
{
    if (_instance == 0)
    {
        _instance = new Params;
    }
    return _instance;
}

/**
 * @brief Updates the parameters using the info obtained from the node handle.
 * @param nh The node handle.
 */
void Params::readParams(const ros::NodeHandle& nh)
{
    // Detector
    std::string str_det;
    nh.param<std::string>("detector", str_det, "SURF");
    ROS_INFO("[Params] Detector: %s", str_det.c_str());
	if (str_det == "SIFT")
	{
		det_detector = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.0);
	}
	else if (str_det == "SURF")
	{
		det_detector = cv::xfeatures2d::SURF::create();
	}
	else
	{
        ROS_ERROR("Cannot create feature detector");
        return;
	}

    // Descriptor
    std::string str_des;
    nh.param<std::string>("descriptor", str_des, "SURF");
    ROS_INFO("[Params] Descriptor: %s", str_des.c_str());
	if (str_det == "SIFT")
	{
		det_descriptor = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.0);
	}
	else if (str_det == "SURF")
	{
		det_descriptor = cv::xfeatures2d::SURF::create();
	}
	else
	{
        ROS_ERROR("Cannot create descriptor extractor");
        return;
	}

    // Debug
    nh.param("debug", debug, false);
    ROS_INFO("[Params] Debug: %s", debug ? "Yes":"No");

    // Reprojection error
    nh.param("rep_error", det_rerror, 3.0);
    ROS_INFO("[Params] Reprojection error: %f", det_rerror);

    // Minimum number of inliers
    nh.param("inliers", det_inliers, 25);
    ROS_INFO("[Params] Number of inliers: %i", det_inliers);

    // Resizing in target detection
    nh.param("resize", det_resize, 1.0);
    ROS_INFO("[Params] Resize in target detection: %f", det_resize);

    // Load target from file?
    nh.param<std::string>("target_from_file", det_target_from_file, "");
    ROS_INFO("[Params] Target from file: %s", det_target_from_file.c_str());

    // Timer for detection phase
    nh.param("det_timer", det_timer, 5.0);
    ROS_INFO("[Params] Detection timer: %f", det_timer);

    // Use camera
    nh.param("use_camera", use_camera, true);
    ROS_INFO("[Params] Use camera: %s", use_camera ? "Yes":"No");

    // Only performs object detection
    nh.param("detect_only", only_detection, false);
    ROS_INFO("[Params] Detection only: %s", only_detection ? "Yes":"No");

    // Tracker
    nh.param<std::string>("tracker", track_tracker, "kcf");
    ROS_INFO("[Params] Tracker: %s", track_tracker.c_str());

    // Threshold to determine whether the tracking is correct
    nh.param("track_thresh", track_thresh, 0.5);
    ROS_INFO("[Params] Tracking threshold: %f", track_thresh);
}
