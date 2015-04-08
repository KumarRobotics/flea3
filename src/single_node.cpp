#include "flea3/single_node.h"

namespace flea3 {

void SingleNode::Acquire() {
  std::cout << "Acquire" << std::endl;
  while (is_acquire() && ros::ok()) {
    flea3_ros_.PublishCamera(ros::Time::now());
    Sleep();
  }
}

void SingleNode::Setup(Config &config) {
  std::cout << "Setup" << std::endl;
  flea3_ros_.Stop();
  flea3_ros_.camera().Configure(config);
  flea3_ros_.set_fps(config.fps);
  flea3_ros_.Start();
}

}  // namespace flea3
