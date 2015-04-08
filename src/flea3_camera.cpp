#include "flea3/flea3_camera.h"

namespace flea3 {

using namespace FlyCapture2;

Flea3Camera::Flea3Camera(const std::string &serial) {

}

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg,
                            sensor_msgs::CameraInfo& cinfo_msg) {}

void HandleError(const Error& error, const std::string& message,
                 const std::string& func_name) {
  if (error == PGRERROR_TIMEOUT) {
    throw std::runtime_error("Failed to retrieve buffer within timeout");
  } else if (error != PGRERROR_OK) {
    const std::string error_type(std::to_string(error.GetType()));
    const std::string error_desc(error.GetDescription());
    throw std::runtime_error(message + " | " + error_type + " " + error_desc +
                             " | " + func_name);
  }
}

}  // namespace flea3
