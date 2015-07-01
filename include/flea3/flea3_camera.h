#ifndef FLEA3_FLEA3_CAMERA_H_
#define FLEA3_FLEA3_CAMERA_H_

#include <flycapture/FlyCapture2.h>
#include "flea3/Flea3DynConfig.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#define PGERROR(err, msg) flea3::HandleError(err, msg, __func__)

using namespace FlyCapture2;

namespace flea3 {

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
  void RequestSingle();
  int expose_us() const;

 private:
  void EnableMetadata();
  void ConnectDevice(PGRGuid* guid);
  void DisconnectDevice();
  std::string AvailableDevice();

  CameraInfo GetCameraInfo();
  Property GetProperty(const PropertyType& prop_type);
  PropertyInfo GetPropertyInfo(const PropertyType& prop_type);

  float GetCameraFrameRate();
  float GetCameraTemperature();

  Mode GetFirstFormat7Mode();
  FrameRate GetMaxFrameRate(const VideoMode& video_mode);
  std::pair<Format7Info, bool> GetFormat7Info(const Mode& mode);
  std::pair<VideoMode, FrameRate> GetVideoModeAndFrameRate();

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

  void SetTriggerMode(bool& enable_trigger);
  bool PollForTriggerReady();
  void FireSoftwareTrigger();

  void SetProperty(const PropertyType& prop_type, bool& auto_on, double& value);
  void SetProperty(const PropertyType& prop_type, double& value);
  void WriteRegister(unsigned address, unsigned value);
  unsigned ReadRegister(unsigned address);

  bool IsVideoModeSupported(const VideoMode& video_mode);
  bool IsFormat7Supported();
  bool IsAutoWhiteBalanceSupported();
  std::pair<Format7PacketInfo, bool> IsFormat7SettingsValid(
      const Format7ImageSettings& fmt7_settings);
  bool IsVideoModeAndFrameRateSupported(const VideoMode& video_mode,
                                        const FrameRate& frame_rate);

  BusManager bus_manager_;
  Camera camera_;
  CameraInfo camera_info_;
  std::string serial_;
  Config config_;
  bool capturing_{false};
  std::vector<double> frame_rates_;
};

void HandleError(const Error& error, const std::string& message = "",
                 const std::string& func_name = "");

void PrintPropertyInfo(const PropertyInfo& prop_info,
                       const std::string& prop_name);
void PrintProperty(const Property& prop, const std::string& prop_name);

std::string BayerFormatToEncoding(const BayerTileFormat& bayer_format);
std::string PixelFormatToEncoding(unsigned bits_per_pixel);

}  // namespace flea3

#endif  // FLEA3_FLEA3_H_
