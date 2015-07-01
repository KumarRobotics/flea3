#include "flea3/single_node.h"

namespace flea3 {

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    flea3_ros_.RequestSingle();
    const auto expose_us = flea3_ros_.camera().expose_us();
    const auto expose_duration = ros::Duration(expose_us * 1e-6 / 2);
    const auto time = ros::Time::now() + expose_duration;
    flea3_ros_.PublishCamera(time);
    Sleep();
  }
}

void SingleNode::Setup(Config &config) {
  flea3_ros_.Stop();
  flea3_ros_.camera().Configure(config);
  flea3_ros_.set_fps(config.fps);
  flea3_ros_.Start();
}

}  // namespace flea3
