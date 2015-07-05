#include "flea3/flea3_ros.h"

namespace flea3 {

Flea3Ros::Flea3Ros(const ros::NodeHandle& nh, const std::string& prefix)
    : CameraRosBase(nh, prefix), flea3_(identifier()) {
  SetHardwareId(flea3_.serial());
}

Flea3Camera& Flea3Ros::camera() { return flea3_; }

bool Flea3Ros::Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msg) {
  return flea3_.GrabImage(*image_msg, *cinfo_msg);
}

void Flea3Ros::Stop() { flea3_.StopCapture(); }

void Flea3Ros::Start() { flea3_.StarCapture(); }

bool Flea3Ros::RequestSingle() { return flea3_.RequestSingle(); }

}  // namespace flea3
