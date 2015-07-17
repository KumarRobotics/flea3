#include "flea3/flea3_camera.h"
#include "flea3/flea3_setting.h"
#include <sensor_msgs/fill_image.h>
#include <utility>

namespace flea3 {

using namespace FlyCapture2;

Flea3Camera::Flea3Camera(const std::string& serial) : serial_(serial) {
  frame_rates_ = {1.875, 3.75, 7.5, 15, 30, 60, 120, 240};
  // Wait for camera to power up
  int num_tries{3};
  while (num_tries > 0) {
    if (Connect()) break;
    usleep(100000);  // Sleep for 100ms
    --num_tries;
  }
  if (num_tries == 0) {
    throw std::runtime_error("Failed after multiple tries, abort.");
  }
}

Flea3Camera::~Flea3Camera() {
  if (camera_.IsConnected())
    PgrError(camera_.Disconnect(), "Failed to disconnect camera");
}

bool Flea3Camera::Connect() {
  PGRGuid guid;
  PgrError(bus_manager_.GetCameraFromSerialNumber(serial_id(), &guid),
           serial_ + " not found. " + AvailableDevice());
  bool success;
  try {
    PgrError(camera_.Connect(&guid), "Failed to connect to camera");
    EnableMetadata(camera_);
    // This is a total hack, it exists because one of my camera doesn't enable
    // auto white balance by default. You have to write to its presence register
    // to bring it online.
    EnableAutoWhiteBalance();
    // For now this only set the grab timeout
    SetConfiguration();
    success = true;
  } catch (const std::exception& e) {
    ROS_INFO("Failed to connect to camera: %s. Try again. | %s",
             serial().c_str(), e.what());
    success = false;
  }
  return success;
}

void Flea3Camera::SetConfiguration() {
  FC2Config config;
  PgrError(camera_.GetConfiguration(&config), "Failed to get configuration");
  // Set the grab timeout to 1 seconds
  //  config.grabTimeout = 1000;
  // Try 2 times before declaring failure
  config.registerTimeoutRetries = 2;
  // Cannot do this here, will block all the following settings on format7
  //  config.highPerformanceRetrieveBuffer = true;

  // Set the camera configuration
  PgrError(camera_.SetConfiguration(&config), "Failed to set configuration");
}

std::string Flea3Camera::AvailableDevice() {
  unsigned num_devices = 0;
  PgrError(bus_manager_.GetNumOfCameras(&num_devices),
           "Failed to get number for cameras");

  std::string devices = std::to_string(num_devices) + " available device(s): ";
  for (unsigned i = 0; i < num_devices; ++i) {
    unsigned serial_id;
    PgrError(bus_manager_.GetCameraSerialNumberFromIndex(i, &serial_id),
             "Failed to get camera serial number from index");
    devices += std::to_string(serial_id) + " ";
  }
  return devices;
}

void Flea3Camera::StarCapture() {
  if (camera_.IsConnected() && !capturing_) {
    PgrError(camera_.StartCapture(), "Failed to start capture");
    capturing_ = true;
  }
}

void Flea3Camera::StopCapture() {
  if (camera_.IsConnected() && capturing_) {
    PgrError(camera_.StopCapture(), "Failed to stop capture");
    capturing_ = false;
  }
}

void Flea3Camera::Configure(Config& config) {
  // Video Mode
  SetVideoMode(config.video_mode, config.format7_mode, config.pixel_format,
               config.width, config.height);

  // Update CameraInfo here after video mode is changed
  camera_info_ = GetCameraInfo(camera_);

  // Frame Rate
  SetFrameRate(config.fps);

  // Exposure
  SetExposure(config.auto_exposure, config.exposure_value);
  SetShutter(config.auto_shutter, config.shutter);
  SetGain(config.auto_gain, config.gain);

  // White Balance
  SetWhiteBalanceRedBlue(config.auto_white_balance, config.wb_red,
                         config.wb_blue);

  SetTriggerMode(config.enable_trigger);

  SetBrightness(config.brightness);
  SetGamma(config.gamma);
  SetRawBayerOutput(config.raw_bayer_output);

  // Save this config
  config_ = config;
}

void Flea3Camera::SetVideoMode(int& video_mode, int& format7_mode,
                               int& pixel_format, int& width, int& height) {
  // Get the default video mode and frame rate for this camera
  const auto curr_video_mode_frame_rate_pg = GetVideoModeAndFrameRate(camera_);

  auto video_mode_pg = static_cast<VideoMode>(video_mode);

  bool is_video_mode_supported;
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    is_video_mode_supported = IsFormat7Supported(camera_);
  } else {
    is_video_mode_supported = IsVideoModeSupported(camera_, video_mode_pg);
  }

  if (!is_video_mode_supported) {
    ROS_WARN_STREAM("Selected video mode not supported: " << video_mode_pg);
    // If the desired video mode is not supported, we fall back to the current
    // video mode, we dont simply return here because at the begining, dynamic
    // reconfigure has no idea what's the available video mode, or if the video
    // mode is format 7
    video_mode_pg = curr_video_mode_frame_rate_pg.first;
  }

  // Actual configuration
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    SetFormat7VideoMode(format7_mode, pixel_format, width, height);
    video_mode = video_mode_pg;
  } else {
    SetStandardVideoMode(video_mode);
    // These configs are not supported in standard video mode
    format7_mode = 0;
    pixel_format = 0;
    width = 0;
    height = 0;
  }
}

void Flea3Camera::SetFormat7VideoMode(int& format7_mode, int& pixel_format,
                                      int& width, int& height) {
  // Check if the required format 7 mode is supported
  auto fmt7_mode_pg = static_cast<Mode>(format7_mode);
  auto fmt7_info = GetFormat7Info(camera_, fmt7_mode_pg);
  if (!fmt7_info.second) {
    ROS_WARN("%s: Format7 mode [%d] is not supported, fall back to [%d]",
             serial().c_str(), format7_mode, MODE_0);
    fmt7_mode_pg = MODE_0;
    fmt7_info = GetFormat7Info(camera_, fmt7_mode_pg);
  }

  Format7ImageSettings fmt7_settings;
  fmt7_settings.mode = fmt7_mode_pg;
  // Set format7 pixel format
  // The 22 here corresponds to PIXEL_FORMAT_RAW8
  pixel_format = (pixel_format == 0) ? 22 : pixel_format;
  fmt7_settings.pixelFormat = static_cast<PixelFormat>(1 << pixel_format);
  // Set format7 ROI
  // Center ROI for now
  SetRoi(fmt7_info.first, fmt7_settings, width, height);

  // Validate the settings
  const auto fmt7_packet_info = IsFormat7SettingsValid(camera_, fmt7_settings);
  if (fmt7_packet_info.second) {
    // TODO: use PgrWarn instead?
    const auto error = camera_.SetFormat7Configuration(
        &fmt7_settings, fmt7_packet_info.first.recommendedBytesPerPacket);
    if (error == PGRERROR_OK) {
      format7_mode = fmt7_mode_pg;
    } else {
      ROS_WARN("%s: Failed to set format7 mode [%d]", serial().c_str(),
               fmt7_mode_pg);
    }
  } else {
    ROS_WARN("Format 7 Setting is not valid");
  }
}

void Flea3Camera::SetStandardVideoMode(int& video_mode) {
  auto video_mode_pg = static_cast<VideoMode>(video_mode);
  const auto max_frame_rate_pg = GetMaxFrameRate(camera_, video_mode_pg);
  PgrWarn(camera_.SetVideoModeAndFrameRate(video_mode_pg, max_frame_rate_pg));

  // Update video mode
  FrameRate frame_rate_pg;
  PgrWarn(camera_.GetVideoModeAndFrameRate(&video_mode_pg, &frame_rate_pg));
  video_mode = video_mode_pg;
}

void Flea3Camera::SetFrameRate(double& frame_rate) {
  // TODO: does not support auto frame rate now
  SetProperty(camera_, FRAME_RATE, frame_rate);
}

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg,
                            sensor_msgs::CameraInfo& cinfo_msg) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  const auto error = camera_.RetrieveBuffer(&image);
  if (error != PGRERROR_OK) return false;

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

void Flea3Camera::SetWhiteBalanceRedBlue(bool& auto_white_balance, int& red,
                                         int& blue) {
  const int default_blue = 800;
  const int default_red = 550;
  if (!camera_info_.isColorCamera) {
    // Not even a color camera, what are you thinking?
    ROS_ERROR("Camera %s is not a color camera, white balance not supported",
              serial().c_str());
    auto_white_balance = false;
    red = default_red;
    blue = default_blue;
    return;
  }

  const unsigned white_balance_addr = 0x80C;
  unsigned value = 1 << 25;  // Enable and turn on

  if (auto_white_balance) {
    if (!IsAutoWhiteBalanceSupported(camera_)) {
      // You want auto white balance, but it is not supported
      ROS_WARN("Auto white balance not supported");
      auto_white_balance = false;
      // Set to some reasonable value
      blue = default_blue;
      red = default_red;
      return;
    }
    value |= 1 << 24;  // Auto
    if (config_.auto_white_balance) {
      const auto prop = GetProperty(camera_, WHITE_BALANCE);
      red = prop.valueA;
      blue = prop.valueB;
    }
  } else {
    value |= blue << 12 | red;
  }
  WriteRegister(camera_, white_balance_addr, value);
}

void Flea3Camera::EnableAutoWhiteBalance() {
  WriteRegister(camera_, 0x80C, 1 << 31);
}

void Flea3Camera::SetExposure(bool& auto_exposure, double& exposure) {
  SetProperty(camera_, AUTO_EXPOSURE, auto_exposure, exposure);
}

void Flea3Camera::SetShutter(bool& auto_shutter, double& shutter) {
  auto shutter_ms = 1000.0 * shutter;
  SetProperty(camera_, SHUTTER, auto_shutter, shutter_ms);
  shutter = shutter_ms / 1000.0;
}

void Flea3Camera::SetGain(bool& auto_gain, double& gain) {
  SetProperty(camera_, GAIN, auto_gain, gain);
}

void Flea3Camera::SetBrightness(double& brightness) {
  SetProperty(camera_, BRIGHTNESS, brightness);
}

void Flea3Camera::SetGamma(double& gamma) {
  SetProperty(camera_, GAMMA, gamma);
}

void Flea3Camera::SetRawBayerOutput(bool& raw_bayer_output) {
  // Because this only works in standard video mode, we only enable this if
  // video mode is not format 7
  if (config_.video_mode == Flea3Dyn_format7) {
    raw_bayer_output = false;
    return;
  }
  // See Point Grey Register Reference document section 5.8
  WriteRegister(camera_, 0x1050, static_cast<unsigned>(raw_bayer_output));
}

void Flea3Camera::SetRoi(const Format7Info& format7_info,
                         Format7ImageSettings& format7_settings, int& width,
                         int& height) {
  format7_settings.offsetX =
      CenterRoi(width, format7_info.maxWidth, format7_info.imageHStepSize);
  format7_settings.width = width;
  format7_settings.offsetY =
      CenterRoi(height, format7_info.maxHeight, format7_info.imageVStepSize);
  format7_settings.height = height;
}

// TODO: Add support for GPIO external trigger
void Flea3Camera::SetTriggerMode(bool& enable_trigger) {
  TriggerModeInfo trigger_mode_info;
  PgrError(camera_.GetTriggerModeInfo(&trigger_mode_info),
           "Failed to get trigger mode info");
  if (!trigger_mode_info.present) {
    // Camera doesn't support external triggering, so set enable_trigger to
    // false
    ROS_WARN("Camera does not support triggering");
    enable_trigger = false;
    return;
  }

  TriggerMode trigger_mode;
  PgrError(camera_.GetTriggerMode(&trigger_mode), "Failed to get trigger mode");
  trigger_mode.onOff = enable_trigger;
  trigger_mode.mode = 0;
  trigger_mode.parameter = 0;
  // Source 7 means software trigger
  trigger_mode.source = 7;
  PgrError(camera_.SetTriggerMode(&trigger_mode), "Failed to set trigger mode");
}

bool Flea3Camera::PollForTriggerReady() {
  const unsigned int software_trigger_addr = 0x62C;
  unsigned int reg_val = 0;

  Error error;
  do {
    error = camera_.ReadRegister(software_trigger_addr, &reg_val);
    if (error != PGRERROR_OK) {
      return false;
    }
  } while ((reg_val >> 31) != 0);

  return true;
}

bool Flea3Camera::FireSoftwareTrigger() {
  const unsigned software_trigger_addr = 0x62C;
  const unsigned fire = 0x80000000;
  const auto error = camera_.WriteRegister(software_trigger_addr, fire);
  return error == PGRERROR_OK;
}

bool Flea3Camera::RequestSingle() {
  if (config_.enable_trigger) {
    if (PollForTriggerReady()) {
      return FireSoftwareTrigger();
    }
  }
  return true;
}

float Flea3Camera::getExposureTimeSec() {
  if (config_.auto_shutter) {
    Property shutter_prop;
    shutter_prop.type = SHUTTER;
    const auto error = camera_.GetProperty(&shutter_prop);
    if (error == PGRERROR_OK) {
      const auto exposure_ms = shutter_prop.absValue;
      return exposure_ms * 1e-3;
    } else {
      return config_.shutter;
    }
  } else {
    return config_.shutter;
  }
}

}  // namespace flea3
