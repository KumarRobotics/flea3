#ifndef FLEA3_FLEA3_CAMERA_H_
#define FLEA3_FLEA3_CAMERA_H_

#include <flycapture/FlyCapture2.h>
#include "flea3/Flea3DynConfig.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace flea3 {
using namespace FlyCapture2;

class Flea3Camera {
 public:
  using Config = ::flea3::Flea3DynConfig;

  explicit Flea3Camera(const std::string& serial);
  ~Flea3Camera();

  const std::string& serial() const { return serial_; }
  const unsigned serial_id() const { return std::atoi(serial_.c_str()); }

  bool GrabImage(sensor_msgs::Image& image_msg,
                 sensor_msgs::CameraInfo& cinfo_msg);

  bool Connect();
  void Configure(Config& config);
  void StarCapture();
  void StopCapture();
  bool RequestSingle();
  float getExposureTimeSec();

 private:
  void ConnectDevice(PGRGuid* guid);
  void DisconnectDevice();
  std::string AvailableDevice();

  void EnableAutoWhiteBalance();
  void SetConfiguration();
  void SetWhiteBalanceRedBlue(bool& auto_white_balance, int& red, int& blue);
  void SetVideoModeAndFrameRateAndFormat7(int& video_mode, double& frame_rate,
                                          int& format7_mode, int& pixel_format);
  void SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                const FrameRate& frame_rate);
  void SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                double& frame_rate);
  void SetFormat7(const Mode& mode, double& frame_rate, int& pixel_format);

  void SetExposure(bool& auto_exposure, double& exposure);
  void SetShutter(bool& auto_shutter, double& shutter);
  void SetGain(bool& auto_gain, double& gain);
  void SetBrightness(double& brightness);
  void SetGamma(double& gamma);
  void SetRawBayerOutput(bool& raw_bayer_output);

  void SetTriggerMode(bool& enable_trigger);
  bool PollForTriggerReady();
  bool FireSoftwareTrigger();

  BusManager bus_manager_;
  Camera camera_;
  CameraInfo camera_info_;
  std::string serial_;
  Config config_;
  bool capturing_{false};
  std::vector<double> frame_rates_;
};

}  // namespace flea3

#endif  // FLEA3_FLEA3_CAMERA_H_
