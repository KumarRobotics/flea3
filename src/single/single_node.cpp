#include "flea3/single_node.h"

namespace flea3 {

SingleNode::SingleNode(const ros::NodeHandle &pnh)
    : CameraNodeBase(pnh), flea3_ros_(pnh) {}

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    if (flea3_ros_.RequestSingle()) {
      const auto expose_duration =
          ros::Duration(flea3_ros_.camera().getExposureTimeSec());
      flea3_ros_.PublishCamera(ros::Time::now() + expose_duration);
      Sleep();
    }
  }
}

void SingleNode::Setup(Config &config) {
  flea3_ros_.Stop();
  flea3_ros_.camera().Configure(config);
  flea3_ros_.set_fps(config.fps);
  flea3_ros_.Start();
}

}  // namespace flea3