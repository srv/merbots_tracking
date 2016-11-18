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

    cv::Mat target = sdata->getTarget();
    phog.describe(target, target_descr);
}

TargetTracker::~TargetTracker()
{
}

void TargetTracker::run()
{
    ros::Rate r(500);
    while (ros::ok())
    {
        if (sdata->getStatus() == TRACKING && sdata->existsImage())
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
                if (distance > 0.6)
                {
                    change_stat = true;
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

        // Sleeping the needed time
        ros::spinOnce();
        r.sleep();
    }
}

void TargetTracker::reset()
{
    first_image = true; // WARNING Mutex?
}
