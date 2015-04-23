#include "flea3/flea3_camera.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace flea3 {

using namespace FlyCapture2;

Flea3Camera::Flea3Camera(const std::string& serial) : serial_(serial) {
  frame_rates_ = {1.875, 3.75, 7.5, 15, 30, 60, 120, 240};
  while (num_tries > 0) {
    if (Connect()) break;
    --num_tries;
    ROS_INFO_STREAM("Try: " << num_tries);
  }
}

Flea3Camera::~Flea3Camera() { DisconnectDevice(); }

bool Flea3Camera::Connect() {
  PGRGuid guid;
  PGERROR(bus_manager_.GetCameraFromSerialNumber(serial_id(), &guid),
          serial_ + " not found. " + AvailableDevice());
  bool success;
  try {
    ConnectDevice(&guid);
    EnableMetadata();
    success = true;
  } catch (const std::exception& e) {
    ROS_INFO("Do I give a fuck?");
    success = false;
  }
  return success;
}

std::string Flea3Camera::AvailableDevice() {
  unsigned num_devices = 0;
  PGERROR(bus_manager_.GetNumOfCameras(&num_devices),
          "Failed to get number for cameras");

  std::string devices = std::to_string(num_devices) + " available device(s): ";
  for (unsigned i = 0; i < num_devices; ++i) {
    unsigned serial_id;
    PGERROR(bus_manager_.GetCameraSerialNumberFromIndex(i, &serial_id),
            "Failed to get camera serial number from index");
    devices += std::to_string(serial_id) + " ";
  }
  return devices;
}

void Flea3Camera::ConnectDevice(PGRGuid* guid) {
  PGERROR(camera_.Connect(guid), "Failed to connect to camera");
}

void Flea3Camera::DisconnectDevice() {
  if (camera_.IsConnected())
    PGERROR(camera_.Disconnect(), "Failed to disconnect camera");
}

void Flea3Camera::StarCapture() {
  if (camera_.IsConnected() && !capturing_) {
    PGERROR(camera_.StartCapture(), "Failed to start capture");
    capturing_ = true;
  }
}

void Flea3Camera::StopCapture() {
  if (camera_.IsConnected() && capturing_) {
    PGERROR(camera_.StopCapture(), "Failed to stop capture");
    capturing_ = false;
  }
}

void Flea3Camera::Configure(Config& config) {
  // Update CameraInfo here
  SetVideoModeAndFrameRateAndFormat7(config.video_mode, config.fps,
                                     config.format7_mode, config.pixel_format);
  camera_info_ = GetCameraInfo();
  SetWhiteBalanceRedBlue(config.auto_white_balance, config.wb_red,
                         config.wb_blue);
  SetExposure(config.auto_exposure, config.exposure);
  SetShutter(config.auto_shutter, config.shutter);
  SetGain(config.auto_gain, config.gain);
  SetBrightness(config.brightness);
  SetGamma(config.gamma);
  config_ = config;
}

void Flea3Camera::EnableMetadata() {
  EmbeddedImageInfo info;
  info.gain.onOff = true;
  info.shutter.onOff = true;
  info.exposure.onOff = true;
  info.timestamp.onOff = true;
  info.brightness.onOff = true;
  info.whiteBalance.onOff = true;
  info.frameCounter.onOff = true;
  PGERROR(camera_.SetEmbeddedImageInfo(&info), "Failed to enable metadata");
}

void Flea3Camera::SetVideoModeAndFrameRateAndFormat7(int& video_mode,
                                                     double& frame_rate,
                                                     int& format7_mode,
                                                     int& pixel_format) {
  // Get the default video mode and frame rate for this camera
  const auto curr_video_mode_frame_rate_pg = GetVideoModeAndFrameRate();
  ROS_INFO_STREAM("curr video mode: " << curr_video_mode_frame_rate_pg.first);
  ROS_INFO_STREAM("curr frame rate: " << curr_video_mode_frame_rate_pg.second);

  auto video_mode_pg = static_cast<VideoMode>(video_mode);

  ROS_INFO("Trying to determine if requested video mode is supported");
  bool video_mode_supported;
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    ROS_INFO_STREAM("Requested video mode is format 7: " << video_mode_pg);
    video_mode_supported = IsFormat7Supported();
  } else {
    ROS_INFO_STREAM("Request video mode is normal: " << video_mode_pg);
    // To see whether this video mode is supported, test it with the lowest
    // frame rate
    video_mode_supported = IsVideoModeSupported(video_mode_pg);
  }

  if (!video_mode_supported) {
    ROS_INFO_STREAM("Selected video mode not supported: " << video_mode_pg);
    // If the desired video mode is not supported, we fall back to the current
    // video mode, we dont simply return here because at the begining, dynamic
    // reconfigure has no idea what's the available video mode, or if the video
    // mode is format 7, what's the available format mode is
    video_mode_pg = curr_video_mode_frame_rate_pg.first;
  }

  ROS_INFO_STREAM("Selected video mode is supported: " << video_mode_pg);
  // Actual configuration
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    ROS_WARN("Format 7 is supported");
    // Check if the request video mode is supported
    auto fmt7_mode_pg = static_cast<Mode>(format7_mode);
    const auto fmt7_info = GetFormat7Info(fmt7_mode_pg);

    if (!fmt7_info.second) {
      ROS_WARN_STREAM(
          "The requested format 7 mode is not valid: " << fmt7_mode_pg);
      fmt7_mode_pg = GetFirstFormat7Mode();
      ROS_WARN_STREAM("Fall back to format 7 mode: " << fmt7_mode_pg);
    }

    ROS_WARN_STREAM("pixel format out: " << pixel_format);
    SetFormat7(fmt7_mode_pg, frame_rate, pixel_format);
    format7_mode = fmt7_mode_pg;
    video_mode = video_mode_pg;
  } else {
    SetVideoModeAndFrameRate(video_mode_pg, frame_rate);
    video_mode = video_mode_pg;
    pixel_format = 0;
  }
}

void Flea3Camera::SetFormat7(const Mode& mode, double& frame_rate,
                             int& pixel_format) {
  const auto fmt7_info = GetFormat7Info(mode);

  Format7ImageSettings fmt7_settings;
  fmt7_settings.mode = mode;
  ROS_WARN_STREAM("Pixel format before: " << pixel_format);
  // The 22 here corresponds to PIXEL_FORMAT_RAW8
  pixel_format = (pixel_format == 0) ? 22 : pixel_format;
  ROS_WARN_STREAM("Pixel format after: " << pixel_format);
  fmt7_settings.pixelFormat = static_cast<PixelFormat>(1 << pixel_format);
  // Just use max for now
  fmt7_settings.width = fmt7_info.first.maxWidth;
  fmt7_settings.height = fmt7_info.first.maxHeight;
  // Validate the settings to make sure that they are valid
  const auto fmt7_packet_info = IsFormat7SettingsValid(fmt7_settings);
  if (fmt7_packet_info.second) {
    ROS_WARN_STREAM("Format 7 valid, packet size: "
                    << fmt7_packet_info.first.recommendedBytesPerPacket);
    PGERROR(
        camera_.SetFormat7Configuration(
            &fmt7_settings, fmt7_packet_info.first.recommendedBytesPerPacket),
        "Failed to set format7 configuration");
    SetProperty(FRAME_RATE, frame_rate);
  } else {
    throw std::runtime_error("Fuck this shit");
  }
}

Mode Flea3Camera::GetFirstFormat7Mode() {
  for (int i = 0; i <= 8; ++i) {
    const auto mode = static_cast<Mode>(i);
    const auto fmt7_info = GetFormat7Info(mode);
    if (fmt7_info.second) {
      return mode;
    }
  }
  // This should never happen
  return {};
}

std::pair<Format7PacketInfo, bool> Flea3Camera::IsFormat7SettingsValid(
    const Format7ImageSettings& fmt7_settings) {
  Format7PacketInfo fmt7_packet_info;
  bool valid;
  PGERROR(camera_.ValidateFormat7Settings(&fmt7_settings, &valid,
                                          &fmt7_packet_info),
          "Failed to validate format7 settings");
  return {fmt7_packet_info, valid};
}

std::pair<Format7Info, bool> Flea3Camera::GetFormat7Info(const Mode& mode) {
  Format7Info fmt7_info;
  bool supported;
  fmt7_info.mode = mode;
  PGERROR(camera_.GetFormat7Info(&fmt7_info, &supported),
          "Failed to get format 7 info");
  return {fmt7_info, supported};
}

bool Flea3Camera::IsFormat7Supported() {
  for (int i = 0; i <= 8; ++i) {
    const auto mode = static_cast<Mode>(i);
    if (GetFormat7Info(mode).second) {
      return true;
    }
  }
  return false;
}

void Flea3Camera::SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                           double& frame_rate) {
  // Get the max frame rate for this video mode
  const auto max_frame_rate_pg = GetMaxFrameRate(video_mode);
  SetVideoModeAndFrameRate(video_mode, max_frame_rate_pg);

  // Set to max supported frame rate and use the config value to fine tune it
  const auto max_frame_rate = frame_rates_[max_frame_rate_pg];
  ROS_INFO_STREAM("Maximum supported frame rate: " << max_frame_rate);
  frame_rate = std::min(frame_rate, max_frame_rate);
  ROS_INFO_STREAM("Set to requested frame rate: " << frame_rate);
  SetProperty(FRAME_RATE, frame_rate);
}

void Flea3Camera::SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                           const FrameRate& frame_rate) {
  PGERROR(camera_.SetVideoModeAndFrameRate(video_mode, frame_rate),
          "Failed to set video mode and frame rate");
}

FrameRate Flea3Camera::GetMaxFrameRate(const VideoMode& video_mode) {
  FrameRate max_frame_rate;
  // This magic number 2 skips VideoMode format 7
  for (int i = NUM_FRAMERATES - 2; i >= 0; --i) {
    const auto frame_rate = static_cast<FrameRate>(i);
    if (IsVideoModeAndFrameRateSupported(video_mode, frame_rate)) {
      max_frame_rate = frame_rate;
      // We return here because there must be one frame rate supported
      break;
    }
  }
  return max_frame_rate;
}

std::pair<VideoMode, FrameRate> Flea3Camera::GetVideoModeAndFrameRate() {
  VideoMode video_mode;
  FrameRate frame_rate;
  PGERROR(camera_.GetVideoModeAndFrameRate(&video_mode, &frame_rate),
          "Failed to get VideoMode and FrameRate");
  return {video_mode, frame_rate};
}

bool Flea3Camera::IsVideoModeAndFrameRateSupported(
    const VideoMode& video_mode, const FrameRate& frame_rate) {
  bool supported;
  PGERROR(
      camera_.GetVideoModeAndFrameRateInfo(video_mode, frame_rate, &supported),
      "Failed to get video mode and frame rate info");
  return supported;
}

bool Flea3Camera::IsVideoModeSupported(const VideoMode& video_mode) {
  return IsVideoModeAndFrameRateSupported(video_mode, FRAMERATE_1_875);
}

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg,
                            sensor_msgs::CameraInfo& cinfo_msg) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  PGERROR(camera_.RetrieveBuffer(&image), "Failed to retrieve buffer");

  // TODO: Change this to use_ros_time?
  if (false) {
    auto time = image.GetTimeStamp();
    image_msg.header.stamp.sec = time.seconds;
    image_msg.header.stamp.nsec = 1000 * time.microSeconds;
    cinfo_msg.header.stamp = image_msg.header.stamp;
  }

  // Set image encodings
  const auto bayer_format = image.GetBayerTileFormat();
  const auto bits_per_pixel = image.GetBitsPerPixel();
  std::string encoding;
  if (camera_info_.isColorCamera) {
    if (bayer_format != NONE) {
      encoding = BayerFormatToEncoding(bayer_format);
    } else if (bits_per_pixel == 24) {
      encoding = sensor_msgs::image_encodings::RGB8;
    } else {
      encoding = PixelFormatToEncoding(bits_per_pixel);
    }
  } else {
    encoding = PixelFormatToEncoding(bits_per_pixel);
  }
  return sensor_msgs::fillImage(image_msg, encoding, image.GetRows(),
                                image.GetCols(), image.GetStride(),
                                image.GetData());
}

CameraInfo Flea3Camera::GetCameraInfo() {
  CameraInfo camera_info;
  PGERROR(camera_.GetCameraInfo(&camera_info), "Failed to get camera info");
  return camera_info;
}

void Flea3Camera::SetWhiteBalanceRedBlue(bool& auto_white_balance, int& red,
                                         int& blue) {
  if (!camera_info_.isColorCamera) {
    // Not even a color camera, what are you thinking?
    ROS_ERROR("Not a color camera");
    auto_white_balance = false;
    red = 0;
    blue = 0;
    return;
  }

  unsigned white_balance_addr = 0x80C;
  unsigned enable = 1 << 31;
  unsigned value = 1 << 25;  // Turn on

  if (auto_white_balance) {
    if (!IsAutoWhiteBalanceSupported()) {
      // You want auto white balance, but it is not supported
      ROS_ERROR("Auto white balance not supported");
      auto_white_balance = false;
      // Set to some reasonable value
      blue = 800;
      red = 550;
      return;
    }
    WriteRegister(white_balance_addr, enable);
    value |= 1 << 24;  // Auto
    if (config_.auto_white_balance) {
      const auto prop = GetProperty(WHITE_BALANCE);
      red = prop.valueA;
      blue = prop.valueB;
    }
  } else {
    value |= blue << 12 | red;
  }
  ROS_ERROR("Change white balance supported");
  WriteRegister(white_balance_addr, value);
}

bool Flea3Camera::IsAutoWhiteBalanceSupported() {
  const auto pinfo = GetPropertyInfo(WHITE_BALANCE);
  return pinfo.autoSupported;
}

void Flea3Camera::SetProperty(const PropertyType& prop_type, bool& auto_on,
                              double& value) {
  auto prop_info = GetPropertyInfo(prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    auto_on = auto_on && prop_info.autoSupported;  // update auto_on
    prop.autoManualMode = auto_on;
    prop.absControl = prop_info.absValSupported;
    prop.onOff = prop_info.onOffSupported;

    value = std::max<double>(std::min<double>(value, prop_info.absMax),
                             prop_info.absMin);
    prop.absValue = value;
    PGERROR(camera_.SetProperty(&prop), "Failed to set property");

    if (auto_on) {
      value = GetProperty(prop_type).absValue;
    }
  }
}

void Flea3Camera::SetProperty(const PropertyType& prop_type, double& value) {
  auto prop_info = GetPropertyInfo(prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    prop.autoManualMode = false;
    prop.absControl = prop_info.absValSupported;
    prop.onOff = prop_info.onOffSupported;
    value = std::max<double>(std::min<double>(value, prop_info.absMax),
                             prop_info.absMin);
    prop.absValue = value;
    PGERROR(camera_.SetProperty(&prop), "Failed to set property");
  }
}

void Flea3Camera::SetExposure(bool& auto_exposure, double& exposure) {
  SetProperty(AUTO_EXPOSURE, auto_exposure, exposure);
}

void Flea3Camera::SetShutter(bool& auto_shutter, double& shutter) {
  auto shutter_ms = 1000.0 * shutter;
  SetProperty(SHUTTER, auto_shutter, shutter_ms);
  shutter = shutter_ms / 1000.0;
}

void Flea3Camera::SetGain(bool& auto_gain, double& gain) {
  SetProperty(GAIN, auto_gain, gain);
}

void Flea3Camera::SetBrightness(double& brightness) {
  SetProperty(BRIGHTNESS, brightness);
}

void Flea3Camera::SetGamma(double& gamma) { SetProperty(GAMMA, gamma); }

void Flea3Camera::WriteRegister(unsigned address, unsigned value) {
  PGERROR(camera_.WriteRegister(address, value), "Failed to write register");
}

float Flea3Camera::GetCameraTemperature() {
  const auto prop = GetProperty(TEMPERATURE);
  // It returns values of 10 * K
  return prop.valueA / 10.0f - 273.15f;
}

float Flea3Camera::GetCameraFrameRate() {
  const auto prop = GetProperty(FRAME_RATE);
  return prop.absValue;
}

Property Flea3Camera::GetProperty(const PropertyType& prop_type) {
  Property prop;
  prop.type = prop_type;
  PGERROR(camera_.GetProperty(&prop), "Failed to get property");
  return prop;
}

PropertyInfo Flea3Camera::GetPropertyInfo(const PropertyType& prop_type) {
  PropertyInfo prop_info;
  prop_info.type = prop_type;
  PGERROR(camera_.GetPropertyInfo(&prop_info), "Failed to get property info");
  return prop_info;
}

std::string BayerFormatToEncoding(const BayerTileFormat& bayer_format) {
  using namespace sensor_msgs::image_encodings;
  switch (bayer_format) {
    case RGGB:
      return BAYER_RGGB8;
    case GRBG:
      return BAYER_GRBG8;
    case GBRG:
      return BAYER_GBRG8;
    case BGGR:
      return BAYER_BGGR8;
    case NONE:
      return MONO8;
    default:
      return MONO8;
  }
}

std::string PixelFormatToEncoding(unsigned bits_per_pixel) {
  using namespace sensor_msgs::image_encodings;
  if (bits_per_pixel == 16) {
    return MONO16;
  } else if (bits_per_pixel == 8) {
    return MONO8;
  }
  return MONO8;
}

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

void printPropertyInfo(const PropertyInfo& prop_info,
                       const std::string& prop_name) {
  std::cout << "* Property Info: " << prop_name;

  if (!prop_info.present) {
    std::cout << " does not exist" << std::endl;
    return;
  }

  std::cout << std::boolalpha;
  std::cout << ", on/off: " << prop_info.onOffSupported
            << ", auto: " << prop_info.autoSupported
            << ", manual: " << prop_info.manualSupported
            << ", one push: " << prop_info.onePushSupported;

  if (!prop_info.readOutSupported) return;

  if (!prop_info.absValSupported) {
    std::cout << ", [abs]"
              << " min: " << prop_info.absMin << ", max: " << prop_info.absMax;
  } else {
    std::cout << ", [int]"
              << " min: " << prop_info.min << ", max: " << prop_info.max;
  }
  std::cout << std::endl;
}

void printProperty(const Property& prop, const std::string& prop_name) {
  std::cout << "* Property: " << prop_name;

  if (!prop.present) {
    std::cout << " does not exist" << std::endl;
    return;
  }

  std::cout << std::boolalpha;
  std::cout << ", on/off: " << prop.onOff << ", auto: " << prop.autoManualMode
            << ", one push: " << prop.onePush << ", value A: " << prop.valueA
            << ", value B: " << prop.valueB << ", abs: " << prop.absValue;

  std::cout << std::endl;
}

}  // namespace flea3
