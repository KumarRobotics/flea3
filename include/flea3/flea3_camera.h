#ifndef FLEA3_FLEA3_CAMERA_H_
#define FLEA3_FLEA3_CAMERA_H_

#include <flycapture/FlyCapture2.h>

#include <boost/thread/mutex.hpp>

namespace flea3 {

class Flea3Camera {
 public:
  explicit Flea3Camera(const std::string& serial);

 private:
  FlyCapture2::BusManager bus_manager_;
  FlyCapture2::Camera camera_;
};

}  // namespace flea3

#endif  // FLEA3_FLEA3_H_
