#include <merbots_tracking/nodelets/det_and_track_nodelet.h>

PLUGINLIB_DECLARE_CLASS(
  merbots_tracking,
  merbots_tracking::ObjectDetectorNodelet,
  merbots_tracking::ObjectDetectorNodelet,
  nodelet::Nodelet);

  namespace merbots_tracking
  {

    void ObjectDetectorNodelet::onInit()
    {
      NODELET_INFO("Initializing Object Detector Nodelet");
      ros::NodeHandle nh = getPrivateNodeHandle();
      objdet_ = new ObjectDetector(nh);
      objdet_->start();
    }

  }
