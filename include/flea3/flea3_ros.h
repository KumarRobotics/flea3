#ifndef FLEA3_FLEA3_ROS_H_
#define FLEA3_FLEA3_ROS_H_

#include "flea3/flea3_camera.h"
#include <camera_base/camera_ros_base.h>

namespace flea3 {

class Flea3Ros : public camera_base::CameraRosBase {
 public:
  Flea3Ros(const ros::NodeHandle& nh,
           const std::string& prefix = std::string());

  Flea3Camera& camera() { return flea3_; }

  bool Grab(const sensor_msgs::ImagePtr& image_msg,
            const sensor_msgs::CameraInfoPtr& cinfo_msg) override;

 private:
  Flea3Camera flea3_;
};

}  // namespace flea3
#endif  // FLEA3_FLEA3_ROS_H_
