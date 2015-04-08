#ifndef FLEA3_FLEA3_CAMERA_H_
#define FLEA3_FLEA3_CAMERA_H_

#include <flycapture/FlyCapture2.h>
#include "flea3/Flea3DynConfig.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#define PGERROR(err, msg) flea3::HandleError(err, msg, __func__)

namespace flea3 {

class Flea3Camera {
 public:
  explicit Flea3Camera(const std::string& serial);

  const std::string& serial() const { return serial_; }
  const unsigned serial_id() const { return std::atoi(serial_.c_str()); }

  bool GrabImage(sensor_msgs::Image& image_msg,
                 sensor_msgs::CameraInfo& cinfo_msg);

  void ConnectDevice();
  std::string AvailableDevice();

 private:
  FlyCapture2::BusManager bus_manager_;
  FlyCapture2::Camera camera_;
  std::string serial_;
};

void HandleError(const FlyCapture2::Error& error,
                 const std::string& message = "",
                 const std::string& func_name = "");
}  // namespace flea3

#endif  // FLEA3_FLEA3_H_
