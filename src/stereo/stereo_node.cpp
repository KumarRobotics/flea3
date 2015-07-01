#include "flea3/stereo_node.h"

namespace flea3 {

void StereoNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    left_ros_.RequestSingle();
    right_ros_.RequestSingle();
    const auto expose_us = left_ros_.camera().expose_us();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    left_ros_.PublishCamera(time);
    right_ros_.PublishCamera(time);
    Sleep();
  }
}

void StereoNode::Setup(Flea3DynConfig &config) {
  left_ros_.set_fps(config.fps);
  right_ros_.set_fps(config.fps);
  auto config_cpy = config;
  left_ros_.camera().Configure(config_cpy);
  right_ros_.camera().Configure(config);
}

}  // namespace flea3
