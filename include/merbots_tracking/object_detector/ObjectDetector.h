#ifndef OBJECT_DETECTOR
#define OBJECT_DETECTOR

#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <ros/ros.h>

#include <merbots_tracking/threads/TargetDetector.h>
#include <merbots_tracking/threads/TargetTracker.h>
#include <merbots_tracking/TargetPoints.h>
#include <merbots_tracking/util/Params.h>
#include <merbots_tracking/util/SharedData.h>
#include <std_msgs/Int32.h>

namespace merbots_tracking
{

  class ObjectDetector
  {
  public:
    ObjectDetector(const ros::NodeHandle& nh_);
    ~ObjectDetector();

    void start();

  private:
    // ROS
    ros::NodeHandle nh;
    // Global parameters
    Params* params;
    SharedData* sdata;
    // Threads
    TargetDetector* tdet;
    TargetTracker* ttrack;
    // Publishers and subscribers
    ros::Publisher roi_pub;
    ros::Publisher inliers_pub;
    image_transport::ImageTransport it;
    image_transport::Subscriber target_sub;
    image_transport::Subscriber img_sub;

    void processImage(const cv::Mat& image);
    void target_cb(const sensor_msgs::ImageConstPtr& msg);
    void image_cb(const sensor_msgs::ImageConstPtr& msg);
    void timer_cb(const ros::TimerEvent &event);
    void publishData();
  };
}

#endif
