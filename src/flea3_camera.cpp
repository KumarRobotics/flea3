#include "flea3/flea3_camera.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace flea3 {

using namespace FlyCapture2;

Flea3Camera::Flea3Camera(const std::string& serial) : serial_(serial) {
  Connect();
}

Flea3Camera::~Flea3Camera() { DisconnectDevice(); }

void Flea3Camera::Connect() {
  PGRGuid guid;
  PGERROR(bus_manager_.GetCameraFromSerialNumber(serial_id(), &guid),
          serial_ + " not found. " + AvailableDevice());
  ConnectDevice(&guid);
  EnableMetadata();
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
  SetVideoModeAndFrameRate(config.video_mode, config.fps);
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

void Flea3Camera::SetVideoModeAndFrameRate(int& video_mode,
                                           double& frame_rate) {
  // Some hack to get things going
  auto pg_frame_rate = FRAMERATE_15;
  auto pg_video_mode = VIDEOMODE_1280x960Y8;

  if (video_mode == flea3::Flea3Dyn_mono8) {
    // MONO8
    pg_video_mode = VIDEOMODE_1280x960Y8;
    if (frame_rate >= 60) {
      frame_rate = 60;
      pg_frame_rate = FRAMERATE_60;
    } else if (frame_rate >= 30) {
      frame_rate = 30;
      pg_frame_rate = FRAMERATE_30;
    } else if (frame_rate >= 15) {
      frame_rate = 15;
      pg_frame_rate = FRAMERATE_15;
    } else if (frame_rate >= 7.5) {
      frame_rate = 7.5;
      pg_frame_rate = FRAMERATE_7_5;
    } else if (frame_rate >= 3.75) {
      frame_rate = 3.75;
      pg_frame_rate = FRAMERATE_3_75;
    } else if (frame_rate >= 1.875) {
      frame_rate = 1.875;
      pg_frame_rate = FRAMERATE_1_875;
    }
  } else if (video_mode == flea3::Flea3Dyn_rgb8) {
    // RGB8
    pg_video_mode = VIDEOMODE_1280x960RGB;
    if (frame_rate >= 30) {
      frame_rate = 30;
      pg_frame_rate = FRAMERATE_30;
    } else if (frame_rate >= 15) {
      frame_rate = 15;
      pg_frame_rate = FRAMERATE_15;
    } else if (frame_rate >= 7.5) {
      frame_rate = 7.5;
      pg_frame_rate = FRAMERATE_7_5;
    } else if (frame_rate >= 3.75) {
      frame_rate = 3.75;
      pg_frame_rate = FRAMERATE_3_75;
    } else if (frame_rate >= 1.875) {
      frame_rate = 1.875;
      pg_frame_rate = FRAMERATE_1_875;
    }
  }
  PGERROR(camera_.SetVideoModeAndFrameRate(pg_video_mode, pg_frame_rate),
          "Failed to set video mode and frame rate");
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

void Flea3Camera::SetWhiteBalanceRedBlue(bool auto_white_balance, int& red,
                                         int& blue) {
  if (camera_info_.isColorCamera) {
    // Register for white balance
    unsigned white_balance_addr = 0x80C;
    unsigned enable = 1 << 31;
    unsigned value = 1 << 25;  // Turn on
    if (auto_white_balance) {
      value |= 1 << 24;  // Auto
    } else {
      value |= blue << 12 | red;
    }

    if (config_.auto_white_balance && auto_white_balance) {
      // Only changes red and blue, just update those values
      const auto prop = GetProperty(WHITE_BALANCE);
      red = prop.valueA;
      blue = prop.valueB;
    }

    // First enable
    WriteRegister(white_balance_addr, enable);
    // Then write to it
    WriteRegister(white_balance_addr, value);
  }
}

void Flea3Camera::SetProperty(const PropertyType& prop_type, bool auto_on,
                              double& value) {
  auto prop_info = GetPropertyInfo(prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    prop.autoManualMode = (auto_on && prop_info.autoSupported);
    prop.absControl = prop_info.absValSupported;
    prop.onOff = prop_info.onOffSupported;

    value = std::max(std::min(value, static_cast<double>(prop_info.absMax)),
                     static_cast<double>(prop_info.absMin));
    prop.absValue = value;
    PGERROR(camera_.SetProperty(&prop), "Failed to set property");

    if (auto_on) {
      value = GetProperty(prop_type).absValue;
    }
  }
}

void Flea3Camera::SetExposure(bool auto_exposure, double& exposure) {
  SetProperty(AUTO_EXPOSURE, auto_exposure, exposure);
}

void Flea3Camera::SetShutter(bool auto_shutter, double& shutter) {
  auto shutter_ms = 1000.0 * shutter;
  SetProperty(SHUTTER, auto_shutter, shutter_ms);
  shutter = shutter_ms / 1000.0;
}

void Flea3Camera::SetGain(bool auto_gain, double& gain) {
  SetProperty(GAIN, auto_gain, gain);
}

void Flea3Camera::SetBrightness(double& brightness) {
  SetProperty(BRIGHTNESS, false, brightness);
}

void Flea3Camera::SetGamma(double& gamma) { SetProperty(GAMMA, false, gamma); }

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
