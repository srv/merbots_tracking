#ifndef OBJDET_NODELET
#define OBJDET_NODELET

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <merbots_tracking/object_detector/ObjectDetector.h>

namespace merbots_tracking
{

  class ObjectDetectorNodelet : public nodelet::Nodelet
  {
  public:
    virtual void onInit ();

  private:
    ObjectDetector* objdet_;
  };

}

#endif
