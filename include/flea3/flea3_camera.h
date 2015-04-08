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
  using Config = ::flea3::Flea3DynConfig;

  explicit Flea3Camera(const std::string& serial);

  const std::string& serial() const { return serial_; }
  const unsigned serial_id() const { return std::atoi(serial_.c_str()); }

  bool GrabImage(sensor_msgs::Image& image_msg,
                 sensor_msgs::CameraInfo& cinfo_msg);

  void Connect();
  void Configure(Config& config);

 private:
  void EnableMetadata();
  void ConnectDevice(FlyCapture2::PGRGuid* guid);
  void DisconnectDevice();
  void StarCapture();
  void StopCapture();
  std::string AvailableDevice();
  FlyCapture2::CameraInfo GetCameraInfo();
  FlyCapture2::Property GetProperty(const FlyCapture2::PropertyType& prop_type);
  FlyCapture2::PropertyInfo GetPropertyInfo(
      const FlyCapture2::PropertyType& prop_type);
  float GetCameraFrameRate();
  float GetCameraTemperature();

  FlyCapture2::BusManager bus_manager_;
  FlyCapture2::Camera camera_;
  FlyCapture2::CameraInfo camera_info_;
  std::string serial_;
  bool capturing_{false};
};

void HandleError(const FlyCapture2::Error& error,
                 const std::string& message = "",
                 const std::string& func_name = "");

void PrintPropertyInfo(const FlyCapture2::PropertyInfo& prop_info,
                       const std::string& prop_name);
void PrintProperty(const FlyCapture2::Property& prop,
                   const std::string& prop_name);

std::string BayerFormatToEncoding(
    const FlyCapture2::BayerTileFormat& bayer_format);
std::string PixelFormatToEncoding(unsigned bits_per_pixel);
}  // namespace flea3

#endif  // FLEA3_FLEA3_H_
