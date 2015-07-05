#include "flea3/flea3_camera.h"
#include "flea3/flea3_setting.h"
#include <sensor_msgs/fill_image.h>

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

Flea3Camera::~Flea3Camera() { DisconnectDevice(); }

bool Flea3Camera::Connect() {
  PGRGuid guid;
  PGERROR(bus_manager_.GetCameraFromSerialNumber(serial_id(), &guid),
          serial_ + " not found. " + AvailableDevice());
  bool success;
  try {
    ConnectDevice(&guid);
    EnableMetadata(camera_);
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
  PGERROR(camera_.GetConfiguration(&config), "Failed to get configuration");
  // Set the grab timeout to 1 seconds
  config.grabTimeout = 1000;

  // Set the camera configuration
  PGERROR(camera_.SetConfiguration(&config), "Failed to set configuration");
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
  SetVideoModeAndFrameRateAndFormat7(config.video_mode, config.fps,
                                     config.format7_mode, config.pixel_format);

  // Update CameraInfo here after video mode is changed
  camera_info_ = GetCameraInfo(camera_);

  // Do other settings
  SetWhiteBalanceRedBlue(config.auto_white_balance, config.wb_red,
                         config.wb_blue);
  SetExposure(config.auto_exposure, config.exposure);
  SetShutter(config.auto_shutter, config.shutter);
  SetGain(config.auto_gain, config.gain);
  SetBrightness(config.brightness);
  SetGamma(config.gamma);
  SetTriggerMode(config.enable_trigger);

  // Save this config
  config_ = config;
}

void Flea3Camera::SetVideoModeAndFrameRateAndFormat7(int& video_mode,
                                                     double& frame_rate,
                                                     int& format7_mode,
                                                     int& pixel_format) {
  // Get the default video mode and frame rate for this camera
  const auto curr_video_mode_frame_rate_pg = GetVideoModeAndFrameRate(camera_);

  auto video_mode_pg = static_cast<VideoMode>(video_mode);

  bool video_mode_supported;
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    video_mode_supported = IsFormat7Supported(camera_);
  } else {
    // To see whether this video mode is supported, test it with the lowest
    // frame rate
    video_mode_supported = IsVideoModeSupported(camera_, video_mode_pg);
  }

  if (!video_mode_supported) {
    ROS_WARN_STREAM("Selected video mode not supported: " << video_mode_pg);
    // If the desired video mode is not supported, we fall back to the current
    // video mode, we dont simply return here because at the begining, dynamic
    // reconfigure has no idea what's the available video mode, or if the video
    // mode is format 7, what's the available format mode is
    video_mode_pg = curr_video_mode_frame_rate_pg.first;
  }

  // Actual configuration
  if (video_mode_pg == VIDEOMODE_FORMAT7) {
    // Check if the request video mode is supported
    auto fmt7_mode_pg = static_cast<Mode>(format7_mode);
    const auto fmt7_info = GetFormat7Info(camera_, fmt7_mode_pg);

    if (!fmt7_info.second) {
      ROS_WARN("The requested format 7 mode [%d] is not valid.", fmt7_mode_pg);
      fmt7_mode_pg = GetFirstFormat7Mode(camera_);
      ROS_WARN("Fall back to format 7 mode [%d]", fmt7_mode_pg);
    }

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
  const auto fmt7_info = GetFormat7Info(camera_, mode);

  Format7ImageSettings fmt7_settings;
  fmt7_settings.mode = mode;
  // The 22 here corresponds to PIXEL_FORMAT_RAW8
  pixel_format = (pixel_format == 0) ? 22 : pixel_format;
  fmt7_settings.pixelFormat = static_cast<PixelFormat>(1 << pixel_format);
  // Just use max for now
  fmt7_settings.width = fmt7_info.first.maxWidth;
  fmt7_settings.height = fmt7_info.first.maxHeight;
  // Validate the settings to make sure that they are valid
  const auto fmt7_packet_info = IsFormat7SettingsValid(camera_, fmt7_settings);
  if (fmt7_packet_info.second) {
    PGERROR(
        camera_.SetFormat7Configuration(
            &fmt7_settings, fmt7_packet_info.first.recommendedBytesPerPacket),
        "Failed to set format7 configuration");
    SetProperty(camera_, FRAME_RATE, frame_rate);
  } else {
    ROS_WARN("Format 7 Settings are not valid");
  }
}

void Flea3Camera::SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                           double& frame_rate) {
  // Get the max frame rate for this video mode
  const auto max_frame_rate_pg = GetMaxFrameRate(camera_, video_mode);
  SetVideoModeAndFrameRate(video_mode, max_frame_rate_pg);

  // Set to max supported frame rate and use the config value to fine tune it
  const auto max_frame_rate = frame_rates_[max_frame_rate_pg];
  frame_rate = std::min(frame_rate, max_frame_rate);
  SetProperty(camera_, FRAME_RATE, frame_rate);
}

void Flea3Camera::SetVideoModeAndFrameRate(const VideoMode& video_mode,
                                           const FrameRate& frame_rate) {
  PGERROR(camera_.SetVideoModeAndFrameRate(video_mode, frame_rate),
          "Failed to set video mode and frame rate");
}

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg,
                            sensor_msgs::CameraInfo& cinfo_msg) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  const auto error = camera_.RetrieveBuffer(&image);
  if (error != PGRERROR_OK) return false;

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

void Flea3Camera::SetWhiteBalanceRedBlue(bool& auto_white_balance, int& red,
                                         int& blue) {
  if (!camera_info_.isColorCamera) {
    // Not even a color camera, what are you thinking?
    ROS_ERROR("Camera %s is not a color camera, white balance not supported",
              serial().c_str());
    auto_white_balance = false;
    red = 0;
    blue = 0;
    return;
  }

  unsigned white_balance_addr = 0x80C;
  unsigned enable = 1 << 31;
  unsigned value = 1 << 25;  // Turn on

  if (auto_white_balance) {
    if (!IsAutoWhiteBalanceSupported(camera_)) {
      // You want auto white balance, but it is not supported
      ROS_WARN("Auto white balance not supported");
      auto_white_balance = false;
      // Set to some reasonable value
      blue = 800;
      red = 550;
      return;
    }
    WriteRegister(camera_, white_balance_addr, enable);
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

// TODO:
void Flea3Camera::SetTriggerMode(bool& enable_trigger) {
  TriggerModeInfo trigger_mode_info;
  PGERROR(camera_.GetTriggerModeInfo(&trigger_mode_info),
          "Failed to get trigger mode info");
  if (!trigger_mode_info.present) {
    // Camera doesn't support external triggering, so set enable_trigger to
    // false
    ROS_WARN("Camera does not support triggering");
    enable_trigger = false;
    return;
  }

  TriggerMode trigger_mode;
  PGERROR(camera_.GetTriggerMode(&trigger_mode), "Failed to get trigger mode");
  trigger_mode.onOff = enable_trigger;
  trigger_mode.mode = 0;
  trigger_mode.parameter = 0;
  // A source of 7 means software trigger
  trigger_mode.source = 7;
  PGERROR(camera_.SetTriggerMode(&trigger_mode), "Failed to set trigger mode");
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
