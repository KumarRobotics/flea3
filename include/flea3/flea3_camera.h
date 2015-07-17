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
  std::string AvailableDevice();

  // Start up
  void EnableAutoWhiteBalance();
  void SetConfiguration();

  // Video Mode
  void SetVideoMode(int& video_mode, int& format7_mode, int& pixel_format,
                    int& width, int& height);
  void SetFormat7VideoMode(int& format7_mode, int& pixel_format, int& width,
                           int& height);
  void SetStandardVideoMode(int& video_mode);
  void SetRoi(const Format7Info& format7_info,
              Format7ImageSettings& format7_settings, int& width, int& height);

  // Frame Rate
  void SetFrameRate(double& frame_rate);

  // White Balance
  void SetWhiteBalanceRedBlue(bool& auto_white_balance, int& red, int& blue);

  void SetExposure(bool& auto_exposure, double& exposure);
  void SetShutter(bool& auto_shutter, double& shutter);
  void SetGain(bool& auto_gain, double& gain);
  void SetBrightness(double& brightness);
  void SetGamma(double& gamma);
  void SetRawBayerOutput(bool& raw_bayer_output);

  // Trigger
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
