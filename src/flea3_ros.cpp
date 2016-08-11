#include "flea3/flea3_ros.h"

namespace flea3 {

Flea3Ros::Flea3Ros(const ros::NodeHandle &pnh, const std::string &prefix)
    : CameraRosBase(pnh, prefix), flea3_(identifier()), pnh_(pnh) {
  SetHardwareId(flea3_.serial());
  pub_image_info_ = cnh().advertise<ImageInfo>("image_info", 1);
  srv_shutter_ms_ =
      cnh().advertiseService("set_shutter_ms", &Flea3Ros::SetShutterMs, this);
}

Flea3Camera &Flea3Ros::camera() { return flea3_; }

bool Flea3Ros::Grab(const sensor_msgs::ImagePtr &image_msg,
                    const sensor_msgs::CameraInfoPtr &cinfo_msg) {
  return flea3_.GrabImage(*image_msg);
}

void Flea3Ros::PublishImageInfo(const ros::Time &time) {
  auto image_info_msg = boost::make_shared<ImageInfo>();
  flea3_.GrabImageInfo(*image_info_msg);
  image_info_msg->header.stamp = time;
  image_info_msg->header.frame_id = frame_id();
  pub_image_info_.publish(image_info_msg);
}

bool Flea3Ros::SetShutterMs(SetShutterMs::Request &req,
                            SetShutterMs::Response &res) {
  ROS_INFO("setting shutter time");
  return true;
}

void Flea3Ros::Stop() { flea3_.StopCapture(); }

void Flea3Ros::Start() { flea3_.StartCapture(); }

bool Flea3Ros::RequestSingle() { return flea3_.RequestSingle(); }

} // namespace flea3
