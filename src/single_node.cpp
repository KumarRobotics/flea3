#include "flea3/single_node.h"

namespace flea3 {

void SingleNode::Acquire() {
  while (is_acquire() && ros::ok()) {
    ROS_INFO("Acquire");
    Sleep();
  }
}

void SingleNode::Setup(Config &config) {
  //
  ROS_INFO("Setup");
}

}  // namespace flea3
