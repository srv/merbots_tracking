#include <merbots_tracking/threads/TargetTracker.h>

TargetTracker::TargetTracker(const ros::NodeHandle& nodeh, Params* params, SharedData* shdata) :
    nh(nodeh),
    p(params),
    sdata(shdata),
    first_image(true)
{    
    if (p->track_tracker == "kcf")
    {
        vtracker = new KCFVisualTracker();
    }
    else if (p->track_tracker == "struck")
    {
        vtracker = new StruckVisualTracker();
    }
    else
    {
        ROS_ERROR("The tracker %s does not exists", p->track_tracker.c_str());
        exit(0);
    }

    if (sdata->existsTarget())
    {
        phog.describe(sdata->getTarget(), target_descr);
    }
}

TargetTracker::~TargetTracker()
{
}

void TargetTracker::run()
{
    ros::Rate r(500);
    while (ros::ok())
    {
        sdata->mutex_upd_target.lock();
        if (sdata->getStatus() == TRACKING && sdata->existsImage() && sdata->existsTarget())
        {
            // Perform the tracking
            cv::Mat image = sdata->getCurrentImage();

            if (first_image)
            {                
                vtracker->initialize(image, sdata->getROI());                
                first_image = false;
            }
            else
            {                
                cv::Rect roi = vtracker->track(image);
                sdata->setROI(roi);
            }

            // Determining if the tracking is correct
            bool change_stat = false;
            cv::Rect curr_roi = sdata->getROI();

            if ((curr_roi.x > 0 && (curr_roi.x + curr_roi.width) < image.cols) &&
                (curr_roi.y > 0 && (curr_roi.y + curr_roi.height) < image.rows))
            {
                cv::Mat roi = image(curr_roi);
                cv::Mat roi_descr;
                phog.describe(roi, roi_descr);
                double distance = PHOG::distChiSquare(target_descr, roi_descr);
                //std::cout << "Distance: " << distance << std::endl;
                if (distance > p->track_thresh)
                {
                    change_stat = true;
                }
                else
                {
                    // We compute the position of the corners of the target
                    int mid_w = static_cast<int>(curr_roi.width / 2.0);
                    int mid_h = static_cast<int>(curr_roi.height / 2.0);

                    // Augmenting the current ROI
                    curr_roi.x = curr_roi.x - mid_w < 0 ?
                                 0 : curr_roi.x - mid_w;
                    curr_roi.y = curr_roi.y - mid_h < 0 ?
                                 0 : curr_roi.y - mid_h;
                    curr_roi.width = (curr_roi.x + (curr_roi.width + mid_w)) > (image.cols - 1)?
                                        (image.cols - curr_roi.x - 1) : (curr_roi.width + mid_w);
                    curr_roi.height = (curr_roi.y + (curr_roi.height + mid_h)) > (image.rows - 1)?
                                        (image.rows - curr_roi.y - 1) : (curr_roi.height + mid_h);

                    // Detecting and describing keypoints of the scene
                    std::vector<cv::KeyPoint> img_kps;
                    p->det_detector->detect(image(curr_roi), img_kps);
                    cv::Mat img_descs;
                    p->det_descriptor->compute(image(curr_roi), img_kps, img_descs);

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
                    }
                    else
                    {
                        change_stat = true;
                    }
                }
            }
            else
            {
                change_stat = true;
            }

            if (change_stat)
            {
                sdata->setStatus(DETECTION);
                reset();
                curr_roi.width = 0;
                curr_roi.height = 0;
                sdata->setROI(curr_roi);
            }
        }
        sdata->mutex_upd_target.unlock();

        // Sleeping the needed time
        ros::spinOnce();
        r.sleep();
    }
}

void TargetTracker::reset()
{
    first_image = true; // WARNING Mutex?
}

void TargetTracker::setTarget()
{
    phog.describe(sdata->getTarget(), target_descr);
    ROS_INFO("Target correctly received in the tracker thread");
}
