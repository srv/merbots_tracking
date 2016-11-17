#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <camera_calibration_parsers/parse_yml.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>

void getImageFilenames(const std::string& directory, std::vector<std::string>& filenames)
{
    using namespace boost::filesystem;

    filenames.clear();
    path dir(directory);

    // Retrieving, sorting and filtering filenames.
    std::vector<path> entries;
    copy(directory_iterator(dir), directory_iterator(), back_inserter(entries));
    sort(entries.begin(), entries.end());
    for (std::vector<path>::const_iterator it(entries.begin()); it != entries.end(); ++it)
    {
        std::string ext = it->extension().c_str();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (ext == ".png" || ext == ".jpg" || ext == ".ppm" || ext == ".jpeg")
        {
            filenames.push_back(it->string());
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "seq_publisher");

    // ROS Communication
    ros::NodeHandle nh("~");

    // Reading parameters
    std::string directory;
    nh.param<std::string>("images_dir", directory, "");
    ROS_INFO("[Param] Directory: %s", directory.c_str());

    std::string camera_info_file;
    nh.param<std::string>("camera_info_file", camera_info_file, "");
    ROS_INFO("[Param] Camera Info File: %s", camera_info_file.c_str());

    double frequency;
    nh.param("freq", frequency, 10.0);
    ROS_INFO("[Param] Frequency: %f", frequency);

    bool publish_ci = false;
    sensor_msgs::CameraInfo cinfo;
    std::string camera_name = "/camera";
    if (camera_info_file != "")
    {
        publish_ci = true;
        // Parsing calibration file
        camera_calibration_parsers::readCalibrationYml(camera_info_file, camera_name, cinfo);
    }

    ROS_INFO("Reading Images ...");
    std::vector<std::string> filenames;
    getImageFilenames(directory, filenames);
    ROS_INFO("Found %lu images", filenames.size());

    // Defining publishers
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_im = it.advertise("image_raw", 300);
    ros::Publisher pub_ci = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    // Iterating for publishing the images
    ros::Rate loop_rate(frequency);
    for (unsigned i = 0; i < filenames.size(); i++)
    {        
        if (ros::isShuttingDown())
        {
            break;
        }

        ROS_INFO("Publishing image %u", i);

        // Preparing the message
        ros::Time now = ros::Time::now();
        cv::Mat image = cv::imread(filenames[i]);

        // Creating Image msg
        cv_bridge::CvImage cvb_image;
        cvb_image.image = image;
        cvb_image.encoding = image.channels() > 1 ? "bgr8" : "mono8";

        sensor_msgs::Image image_msg;
        cvb_image.toImageMsg(image_msg);
        image_msg.header.stamp = now;
        image_msg.header.frame_id = camera_name;
        pub_im.publish(image_msg);

        if (publish_ci)
        {
            cinfo.header.stamp = now;
            cinfo.header.frame_id = camera_name;
            pub_ci.publish(cinfo);
        }

		ros::spinOnce();
        loop_rate.sleep();
	}

    ROS_INFO("All images have been published");

	return 0;
}
