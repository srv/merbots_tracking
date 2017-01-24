#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

class TargetSelector
{
public:
    enum Mode
    {
        DISPLAY,
        WAIT_IMAGE,
        SELECT_IN,
        SELECT_OUT
    };

    TargetSelector() :
            nh("~"),
            it(nh),
            curr_mode(DISPLAY),
            curr_roi(0, 0, 0, 0)
    {
        img_sub = it.subscribe("image", 0, &TargetSelector::image_cb, this);
        target_pub = it.advertise("target", 0);

        cv::namedWindow("Target Selector");
        cv::setMouseCallback("Target Selector", &TargetSelector::staticMouse_cb, this);

        timer_repaint = nh.createTimer(ros::Duration(0.05), &TargetSelector::timerRepaint_cb, this);

        ROS_INFO("-Target Selector-");
        ROS_INFO("Please, click left to select the desired ROI");
    }

    ~TargetSelector()
    {
        cv::destroyWindow("Target Selector");
    }

private:

    // Image ROS topic callback
    void image_cb(const sensor_msgs::ImageConstPtr& msg)
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

        cv_ptr->image.copyTo(curr_image);
        if (curr_mode == WAIT_IMAGE)
        {
            cv_ptr->image.copyTo(curr_target);
            curr_mode = SELECT_IN;
        }
    }

    static void staticMouse_cb(int event, int x, int y, int flags, void* param)
    {
        // Extract this pointer and call function on object
        TargetSelector* node = reinterpret_cast<TargetSelector*>(param);
        assert(node != NULL);
        node->mouse_cb(event, x, y, flags);
    }

    void mouse_cb(int event, int x, int y, int flags)
    {
        curr_mousepos.x = x;
        curr_mousepos.y = y;

        //ROS_INFO("%i, %i, Mode: %i", x, y, curr_mode);

        if (curr_mode == DISPLAY)
        {
            if (event == CV_EVENT_LBUTTONDOWN)
            {
                ROS_INFO("Selecting the target ...");
                curr_mode = WAIT_IMAGE;
            }
        }
        else if (curr_mode == SELECT_IN)
        {
            if (event == CV_EVENT_LBUTTONDOWN)
            {
                initpos.x = x;
                initpos.y = y;
                curr_mode = SELECT_OUT;
            }
        }
        else if (curr_mode == SELECT_OUT)
        {
            if (event == CV_EVENT_LBUTTONDOWN)
            {

                // Obtaining the selected ROI
                int xmin = std::min(initpos.x, curr_mousepos.x);
                int xmax = std::max(initpos.x, curr_mousepos.x);
                int ymin = std::min(initpos.y, curr_mousepos.y);
                int ymax = std::max(initpos.y, curr_mousepos.y);
                curr_roi = cv::Rect(xmin, ymin, xmax - xmin, ymax - ymin);
                //ROS_INFO("%i, %i, %i, %i", xmin, ymin, xmax, ymax);

                // Getting the image
                cv::Mat img;
                curr_target(curr_roi).copyTo(img);

                // Converting the image to sensor_msgs::ImagePtr
                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

                // Publishing the new target
                target_pub.publish(msg);

                curr_mode = DISPLAY;
            }
        }
    }

    // Repaint function
    void timerRepaint_cb(const ros::TimerEvent &event)
    {
        if (curr_image.empty() && curr_target.empty())
        {
            return;
        }

        // Repainting the main window
        cv::Mat img;
        if (curr_mode == DISPLAY || curr_mode == WAIT_IMAGE)
        {
            curr_image.copyTo(img);
        }
        else
        {
            curr_target.copyTo(img);
            if (curr_mode == SELECT_OUT)
            {
              cv::rectangle(img, initpos, curr_mousepos, cv::Scalar(0, 255, 0), 3);
            }
        }
        cv::imshow("Target Selector", img);
        cv::waitKey(5);
    }

    // Variables
    Mode curr_mode;
    cv::Mat curr_target;
    cv::Mat curr_image;
    cv::Rect curr_roi;
    cv::Point curr_mousepos;
    cv::Point initpos;

    // ROS
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    image_transport::Subscriber img_sub;
    image_transport::Publisher target_pub;
    ros::Timer timer_repaint;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "target_selector");
    TargetSelector tsel;
    ros::spin();
    return 0;
}
