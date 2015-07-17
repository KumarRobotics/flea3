#include "flea3/flea3_setting.h"
#include <sensor_msgs/image_encodings.h>
#include <iostream>
#include <ros/ros.h>

namespace flea3 {

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

void PgrError(const Error& error, const std::string& message) {
  if (error == PGRERROR_TIMEOUT) {
    throw std::runtime_error("Failed to retrieve buffer within timeout");
  } else if (error != PGRERROR_OK) {
    throw std::runtime_error(message + " | " + std::to_string(error.GetType()) +
                             " " + error.GetDescription());
  }
}

bool PgrWarn(const Error& error, const std::string& message) {
  if (error != PGRERROR_OK) {
    ROS_WARN("%s, %s", error.GetDescription(), message.c_str());
    return false;
  } else {
    return true;
  }
}

void PrintPropertyInfo(const PropertyInfo& prop_info,
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

  if (prop_info.absValSupported) {
    std::cout << ", [abs]"
              << " min: " << prop_info.absMin << ", max: " << prop_info.absMax;
  } else {
    std::cout << ", [int]"
              << " min: " << prop_info.min << ", max: " << prop_info.max;
  }
  std::cout << std::endl;
}

void PrintProperty(const Property& prop, const std::string& prop_name) {
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

unsigned ReadRegister(Camera& camera, unsigned address) {
  unsigned reg_val;
  PgrError(camera.ReadRegister(address, &reg_val), "Failed to read register");
  return reg_val;
}

PropertyInfo GetPropertyInfo(Camera& camera, const PropertyType& prop_type) {
  PropertyInfo prop_info;
  prop_info.type = prop_type;
  PgrError(camera.GetPropertyInfo(&prop_info), "Failed to get property info");
  return prop_info;
}

Property GetProperty(Camera& camera, const PropertyType& prop_type) {
  Property prop;
  prop.type = prop_type;
  PgrError(camera.GetProperty(&prop), "Failed to get property");
  return prop;
}

std::pair<Format7Info, bool> GetFormat7Info(Camera& camera, const Mode& mode) {
  Format7Info fmt7_info;
  bool supported = false;
  fmt7_info.mode = mode;
  PgrWarn(camera.GetFormat7Info(&fmt7_info, &supported),
          "Failed to get format 7 info");
  return {fmt7_info, supported};
}

CameraInfo GetCameraInfo(Camera& camera) {
  CameraInfo camera_info;
  PgrError(camera.GetCameraInfo(&camera_info), "Failed to get camera info");
  return camera_info;
}

float GetCameraFrameRate(Camera& camera) {
  const auto prop = GetProperty(camera, FRAME_RATE);
  return prop.absValue;
}

FrameRate GetMaxFrameRate(Camera& camera, const VideoMode& video_mode) {
  // This magic number 2 skips VideoMode format 7
  for (int i = NUM_FRAMERATES - 2; i >= 0; --i) {
    const auto frame_rate = static_cast<FrameRate>(i);
    if (IsVideoModeAndFrameRateSupported(camera, video_mode, frame_rate)) {
      // Whatever we get here should be the supported max frame rate
      return frame_rate;
    }
  }
  return FRAMERATE_1_875;
}

std::pair<VideoMode, FrameRate> GetVideoModeAndFrameRate(Camera& camera) {
  VideoMode video_mode;
  FrameRate frame_rate;
  PgrError(camera.GetVideoModeAndFrameRate(&video_mode, &frame_rate),
           "Failed to get VideoMode and FrameRate");
  return {video_mode, frame_rate};
}

float GetCameraTemperature(Camera& camera) {
  const auto prop = GetProperty(camera, TEMPERATURE);
  // It returns values of 10 * K
  return prop.valueA / 10.0f - 273.15f;
}

void SetProperty(Camera& camera, const PropertyType& prop_type, bool& on,
                 bool& auto_on, double& value) {
  auto prop_info = GetPropertyInfo(camera, prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    prop.autoManualMode = auto_on;
    prop.absControl = prop_info.absValSupported;
    prop.onOff = on;
    prop.absValue = value;
    PgrWarn(camera.SetProperty(&prop),
            "Failed to set property: " + std::to_string(prop_type));

    // Validate and update value
    PgrWarn(camera.GetProperty(&prop),
            "Failed to get property: " + std::to_string(prop_type));
    on = prop.onOff;
    if (prop.onOff) {
      value = prop.absValue;
      auto_on = prop.autoManualMode;
    }
  }
}

void SetProperty(Camera& camera, const PropertyType& prop_type, bool& auto_on,
                 double& value) {
  auto prop_info = GetPropertyInfo(camera, prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    prop.autoManualMode = auto_on;
    prop.absControl = prop_info.absValSupported;
    prop.onOff = prop_info.onOffSupported;
    prop.absValue = value;
    PgrWarn(camera.SetProperty(&prop),
            "Failed to set property: " + std::to_string(prop_type));

    // Validate and update value
    PgrWarn(camera.GetProperty(&prop),
            "Failed to get property: " + std::to_string(prop_type));
    if (prop.onOff) {
      value = prop.absValue;
      auto_on = prop.autoManualMode;
    }
  }
}

void SetProperty(Camera& camera, const PropertyType& prop_type, double& value) {
  const auto prop_info = GetPropertyInfo(camera, prop_type);
  if (prop_info.present) {
    Property prop;
    prop.type = prop_type;
    prop.autoManualMode = false;
    prop.absControl = prop_info.absValSupported;
    prop.onOff = prop_info.onOffSupported;
    prop.absValue = value;
    PgrWarn(camera.SetProperty(&prop),
            "Failed to set property: " + std::to_string(prop_type));

    // Validate and update value
    PgrWarn(camera.GetProperty(&prop),
            "Failed to get property" + std::to_string(prop_type));
    if (prop.onOff) {
      value = prop.absValue;
    }
  }
}

void WriteRegister(Camera& camera, unsigned address, unsigned value) {
  PgrWarn(camera.WriteRegister(address, value), "Failed to write register");
}

void EnableMetadata(Camera& camera) {
  EmbeddedImageInfo info;
  info.gain.onOff = true;
  info.shutter.onOff = true;
  info.exposure.onOff = true;
  info.timestamp.onOff = true;
  info.brightness.onOff = true;
  info.whiteBalance.onOff = true;
  info.frameCounter.onOff = true;
  PgrWarn(camera.SetEmbeddedImageInfo(&info), "Failed to enable metadata");
}

bool IsAutoWhiteBalanceSupported(Camera& camera) {
  const auto pinfo = GetPropertyInfo(camera, WHITE_BALANCE);
  return pinfo.autoSupported;
}

bool IsVideoModeSupported(Camera& camera, const VideoMode& video_mode) {
  return IsVideoModeAndFrameRateSupported(camera, video_mode, FRAMERATE_1_875);
}

bool IsVideoModeAndFrameRateSupported(Camera& camera,
                                      const VideoMode& video_mode,
                                      const FrameRate& frame_rate) {
  bool supported = false;
  PgrWarn(
      camera.GetVideoModeAndFrameRateInfo(video_mode, frame_rate, &supported),
      "Failed to get video mode and frame rate info");
  return supported;
}

bool IsFormat7Supported(Camera& camera) {
  const int num_modes{NUM_MODES};
  for (int i = 0; i < num_modes; ++i) {
    const auto mode = static_cast<Mode>(i);
    if (GetFormat7Info(camera, mode).second) {
      // Supported
      return true;
    }
  }
  return false;
}

std::pair<Format7PacketInfo, bool> IsFormat7SettingsValid(
    Camera& camera, Format7ImageSettings& fmt7_settings) {
  Format7PacketInfo fmt7_packet_info;
  bool valid = false;
  PgrWarn(
      camera.ValidateFormat7Settings(&fmt7_settings, &valid, &fmt7_packet_info),
      "Failed to validate format7 settings");
  return {fmt7_packet_info, valid};
}

int CenterRoi(int& size, int max_size, int step) {
  if (size == 0 || size > max_size) size = max_size;
  // size should be a multiple of step
  size = size / step * step;
  // Return offset for centering roi
  return (max_size - size) / 2;
}

}  // namespace flea3
