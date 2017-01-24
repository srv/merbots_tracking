#include <merbots_tracking/object_detector/ObjectDetector.h>

namespace merbots_tracking
{
  ObjectDetector::ObjectDetector(const ros::NodeHandle& nh_) :
    nh(nh_),
    it(nh_),
    tdet(0),
    ttrack(0)
  {
    ROS_INFO("MERBOTS detection and tracking");

    // Reading parameters
    ROS_INFO("Reading parameters ...");
    params = Params::getInstance();
    params->readParams(nh);
    ROS_INFO("Parameters read");

    // Initializing shared data
    ROS_INFO("Initializing shared data ...");
    sdata = SharedData::getInstance();
    ROS_INFO("Shared data initialized");

    // Topic for setting the target
    target_sub = it.subscribe("target", 0, &ObjectDetector::target_cb, this);

    // Region of Interest publisher
    roi_pub = nh.advertise<merbots_tracking::TargetPoints>("roi", 1);

    // Inliners publisher
    inliers_pub = nh.advertise<std_msgs::Int32>("inliers", 1);

    // Window to show the results
    if (params->debug)
    {
      cv::namedWindow("Detection and Tracking");
      cv::namedWindow("Current Target");
    }

    // Threads
    // Target Detector Thread
    tdet = new TargetDetector(nh, params, sdata);
    boost::thread tdet_thread(&TargetDetector::run, tdet);

    // Target Tracker Thread
    ttrack = new TargetTracker(nh, params, sdata);
    boost::thread ttrack_thread(&TargetTracker::run, ttrack);

    if (params->det_target_from_file != "")
    {
      cv::Mat image = cv::imread(params->det_target_from_file);

      sdata->mutex_upd_target.lock();
      sdata->setTarget(image);
      //tdet->setTarget();
      ttrack->setTarget();
      sdata->mutex_upd_target.unlock();

      ROS_INFO("Target loaded from: %s", params->det_target_from_file.c_str());
    }

    ros::Timer timer_det = nh.createTimer(ros::Duration(params->det_timer), &ObjectDetector::timer_cb, this);
  }

  ObjectDetector::~ObjectDetector()
  {
    if (tdet) delete tdet;
    if (ttrack) delete ttrack;
  }

  void ObjectDetector::start()
  {
    // Processing images
    if (params->use_camera)
    {
      // Use an USB camera to receive images
      cv::Mat curr_image;
      int key;
      cv::VideoCapture cap = cv::VideoCapture(0);
      while(ros::ok())
      {
        cap >> curr_image;
        processImage(curr_image);

        key = cv::waitKey(10);
        if (key == 27)
        {
          break;
        }

        publishData();
      }
    }
    else
    {
      // Use a ROS topic to receive images

      // Publishers and Subscribers
      img_sub = it.subscribe("image", 0, &ObjectDetector::image_cb, this);

      // Spinning the ROS execution
      while (ros::ok())
      {
        publishData();
        ros::spinOnce();
      }
    }
  }

  void ObjectDetector::processImage(const cv::Mat& image)
  {
    cv::Mat timg;
    cv::resize(image, timg, cv::Size(), params->det_resize, params->det_resize);

    sdata->setCurrentImage(timg);
  }

  // Target callback
  void ObjectDetector::target_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Converting the image message to OpenCV format
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Error converting image to OpenCV format: %s", e.what());
      return;
    }

    // Update the state
    sdata->mutex_upd_target.lock();
    sdata->setTarget(cv_ptr->image);
    //tdet->setTarget();
    ttrack->setTarget();
    if (sdata->getStatus() == TRACKING)
    {
      sdata->setStatus(DETECTION);
      ttrack->reset();
    }
    sdata->mutex_upd_target.unlock();

    // Cleaning the current roi
    cv::Rect roi;
    roi.width = 0;
    roi.height = 0;
    sdata->setROI(roi);
  }

  // Image ROS topic callback
  void ObjectDetector::image_cb(const sensor_msgs::ImageConstPtr& msg)
  {
    // Converting the image message to OpenCV format
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Error converting image to OpenCV format: %s", e.what());
      return;
    }

    processImage(cv_ptr->image);
  }

  // Detection phase timer
  void ObjectDetector::timer_cb(const ros::TimerEvent &event)
  {
    // Update the state
    if (sdata->getStatus() == TRACKING)
    {
      sdata->setStatus(DETECTION);
      ttrack->reset();
    }

    // Cleaning the current roi
    cv::Rect roi;
    roi.width = 0;
    roi.height = 0;
    sdata->setROI(roi);
  }

  void ObjectDetector::publishData()
  {
    // Publishing the current ROI
    cv::Rect roi = sdata->getROI();
    bool existROI = roi.width > 0 && roi.height > 0;

    merbots_tracking::TargetPointsPtr roi_msg(new merbots_tracking::TargetPoints);
    std::vector<cv::Point2f> pts;

    // Publishing ROI
    if (existROI)
    {
      sdata->getScenePoints(pts);

      roi_msg->point_tl.x = pts[0].x * params->det_resize_inv;
      roi_msg->point_tl.y = pts[0].y * params->det_resize_inv;
      roi_msg->point_tr.x = pts[1].x * params->det_resize_inv;
      roi_msg->point_tr.y = pts[1].y * params->det_resize_inv;
      roi_msg->point_bl.x = pts[2].x * params->det_resize_inv;
      roi_msg->point_bl.y = pts[2].y * params->det_resize_inv;
    }

    // Publishing ROI
    roi_msg->exists_roi = (unsigned char) (existROI ? 1 : 0);
    roi_pub.publish(roi_msg);

    // Publishing Inliers
    int inliers = sdata->getInliers();
    std_msgs::Int32Ptr inliers_msg(new std_msgs::Int32);
    inliers_msg->data = inliers;
    inliers_pub.publish(inliers_msg);

    // Publishing the image if needed
    if (params->debug && sdata->existsImage() && sdata->existsTarget())
    {
      cv::Mat img, tgt;
      sdata->copyCurrentImage(img);
      sdata->copyCurrentTarget(tgt);

      // Plotting the working mode in the image
      if (sdata->getStatus() == DETECTION)
      {
        cv::putText(img, "D", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
      }
      else
      {
        cv::putText(img, "T", cv::Point(30,30), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(0,255,0), 1, CV_AA);
      }

      if (existROI)
      {
        //            cv::rectangle(img, roi, cv::Scalar(0,0, 255), 2);

        cv::circle(img, cv::Point((int)pts[0].x, (int)pts[0].y), 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, cv::Point((int)pts[1].x, (int)pts[1].y), 3, cv::Scalar(0, 255, 0), -1);
        cv::circle(img, cv::Point((int)pts[2].x, (int)pts[2].y), 3, cv::Scalar(0, 255, 0), -1);
      }

      cv::imshow("Detection and Tracking", img);
      cv::imshow("Current Target", tgt);
      cv::waitKey(5);
    }
  }
}
