#include <merbots_tracking/threads/TargetDetector.h>

TargetDetector::TargetDetector(const ros::NodeHandle& nodeh, Params* params, SharedData* shdata) :
    nh(nodeh),
    p(params),
    sdata(shdata)
{
}

TargetDetector::~TargetDetector()
{
}

void TargetDetector::run()
{
    ros::Rate r(500);
    while (ros::ok())
    {
        sdata->mutex_upd_target.lock();
        if (sdata->getStatus() == DETECTION && sdata->existsImage() && sdata->existsTarget())
        {
            // Computes the detection
            cv::Mat image = sdata->getCurrentImage();

            // Detecting and describing keypoints of the scene
            std::vector<cv::KeyPoint> img_kps;
            p->det_detector->detect(image, img_kps);
            cv::Mat img_descs;
            p->det_descriptor->compute(image, img_kps, img_descs);

            // Matching descriptors of the current scene against the target
            cv::Mat results;
            cv::Mat dists;
            sdata->searchKeypoints(img_descs, results, dists, 2);

            // Filtering matchings using the NNDR
            std::vector<cv::Point2f> mpts_obj, mpts_img;
            for (unsigned int i = 0; i < img_descs.rows; i++)
            {
                if (results.at<int>(i, 0) >= 0 && results.at<int>(i, 1) >= 0
                    && dists.at<float>(i, 0) < 0.8 * dists.at<float>(i, 1))
                {
                    mpts_img.push_back(img_kps[i].pt);
                    mpts_obj.push_back(sdata->getKeypoint(results.at<int>(i, 0)));
                }
            }

            // Estimating the homography
            bool thereis_object = false;
            cv::Mat homography;
            cv::Mat ninliers;
            if (mpts_img.size() > 7)
            {
                homography = cv::findHomography(mpts_obj, mpts_img, cv::RANSAC, p->det_rerror, ninliers);

                // Counting the number of inliers
                int total_inliers = 0;
                for (int i = 0; i < ninliers.rows; i++)
                {
                    if ((unsigned int)ninliers.at<uchar>(i))
                    {
                        total_inliers++;
                    }
                }

                //ROS_INFO("Inliers %i", total_inliers);

                if (!homography.empty() && total_inliers > p->det_inliers)
                {
                    thereis_object = true;
                }
            }

            // If the object is present in the image
            if (thereis_object)
            {
                // Transforming points
                std::vector<cv::Point2f> obj_corners(4);
                sdata->getTargetPoints(obj_corners);
                std::vector<cv::Point2f> scene_corners(4);
                cv::perspectiveTransform(obj_corners, scene_corners, homography);

                // Storing the transformed points
                sdata->setScenePoints(scene_corners);

                std::vector<int> xlist, ylist;
                xlist.push_back(static_cast<int>(scene_corners[0].x));
                xlist.push_back(static_cast<int>(scene_corners[1].x));
                xlist.push_back(static_cast<int>(scene_corners[2].x));
                xlist.push_back(static_cast<int>(scene_corners[3].x));
                ylist.push_back(static_cast<int>(scene_corners[0].y));
                ylist.push_back(static_cast<int>(scene_corners[1].y));
                ylist.push_back(static_cast<int>(scene_corners[2].y));
                ylist.push_back(static_cast<int>(scene_corners[3].y));

                // Determining the bounding box
                int xmin = std::max(0, *std::min_element(xlist.begin(), xlist.end()));
                int xmax = std::min(image.cols, *std::max_element(xlist.begin(), xlist.end()));
                int ymin = std::max(0, *std::min_element(ylist.begin(), ylist.end()));
                int ymax = std::min(image.rows, *std::max_element(ylist.begin(), ylist.end()));

                // Setting the current ROI
                sdata->setROI(xmin, ymin, xmax - xmin, ymax - ymin);

                // Changing to tracking, if needed
                if (!p->only_detection)
                {
                    sdata->setStatus(TRACKING);
                }
            }
            else
            {
                cv::Rect roi = sdata->getROI();
                if (roi.width != 0 || roi.height != 0)
                {
                    // Cleaning the current ROI
                    roi.width = 0;
                    roi.height = 0;
                    sdata->setROI(roi);
                }
            }
        }
        sdata->mutex_upd_target.unlock();

        // Sleeping the needed time
        ros::spinOnce();
        r.sleep();
    }
}