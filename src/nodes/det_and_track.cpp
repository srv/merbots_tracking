#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <ros/ros.h>
#include <sensor_msgs/RegionOfInterest.h>

#include <merbots_tracking/threads/TargetDetector.h>
#include <merbots_tracking/threads/TargetTracker.h>
#include <merbots_tracking/util/Params.h>
#include <merbots_tracking/util/SharedData.h>

// Global parameters
Params* params;
SharedData* sdata;

// Public publishers and subscribers
ros::Publisher roi_pub;

void processImage(const cv::Mat& image)
{
    cv::Mat timg;
    cv::resize(image, timg, cv::Size(), 1.0, 1.0);

    //sdata->setCurrentImage(image);
    sdata->setCurrentImage(timg);

    // Publishing the current ROI
    cv::Rect roi = sdata->getROI();
    bool existROI = roi.width > 0 && roi.height > 0;
    if (existROI)
    {
        sensor_msgs::RegionOfInterest roi_msg;
        roi_msg.x_offset = static_cast<unsigned>(roi.x);
        roi_msg.y_offset = static_cast<unsigned>(roi.y);
        roi_msg.width = static_cast<unsigned>(roi.width);
        roi_msg.height = static_cast<unsigned>(roi.height);
        roi_msg.do_rectify = 1;
        roi_pub.publish(roi_msg);
    }

    // Publishing the image if needed
    if (params->debug)
    {
        cv::Mat img;
        timg.copyTo(img);

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
            cv::rectangle(img, roi, cv::Scalar(0,0, 255), 2);
        }
        cv::imshow("Detection and Tracking", img);
        cv::waitKey(5);
    }
}

// Image ROS topic callback
void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    // Converting the image message to OpenCV format
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Error converting image to OpenCV format: %s", e.what());
        return;
    }

    processImage(cv_ptr->image);
}

// Main function
int main(int argc, char** argv)
{
    // ROS
    ros::init(argc, argv, "det_and_track");
    ros::NodeHandle nh("~");

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

    if (params->det_target_from_file != "")
    {
        cv::Mat image = cv::imread(params->det_target_from_file);
        sdata->setTarget(image);
        ROS_INFO("Target loaded from: %s", params->det_target_from_file.c_str());
    }

    // TODO Add set_target service

    // Region of Interest publisher
    roi_pub = nh.advertise<sensor_msgs::RegionOfInterest>("target", 1);

    // Window to show the results
    if (params->debug)
    {
        cv::namedWindow("Detection and Tracking");
    }

    // Threads
    // Target Detector Thread
    TargetDetector tdet(nh, params, sdata);
    boost::thread tdet_thread(&TargetDetector::run, &tdet);

    // Target Tracker Thread
    TargetTracker ttrack(nh, params, sdata);
    boost::thread ttrack_thread(&TargetTracker::run, &ttrack);

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
        }
    }
    else
    {
        // Use a ROS topic to receive images

        // Publishers and Subscribers
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber img_sub = it.subscribe("image", 0, image_callback);

        // Spinning the ROS execution
        ros::spin();
    }

    return 0;
}