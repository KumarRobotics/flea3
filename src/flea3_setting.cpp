#include "flea3/flea3_setting.h"
#include <sensor_msgs/image_encodings.h>
#include <iostream>

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

unsigned ReadRegister(Camera& camera, unsigned address) {
  unsigned reg_val;
  PGERROR(camera.ReadRegister(address, &reg_val), "Failed to read register");
  return reg_val;
}

PropertyInfo GetPropertyInfo(Camera& camera, const PropertyType& prop_type) {
  PropertyInfo prop_info;
  prop_info.type = prop_type;
  PGERROR(camera.GetPropertyInfo(&prop_info), "Failed to get property info");
  return prop_info;
}

Property GetProperty(Camera& camera, const PropertyType& prop_type) {
  Property prop;
  prop.type = prop_type;
  PGERROR(camera.GetProperty(&prop), "Failed to get property");
  return prop;
}

void SetProperty(Camera& camera, const PropertyType& prop_type, bool& auto_on,
                 double& value) {
  auto prop_info = GetPropertyInfo(camera, prop_type);
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
    PGERROR(camera.SetProperty(&prop), "Failed to set property");

    // Update value
    if (auto_on) value = GetProperty(camera, prop_type).absValue;
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
    value = std::max<double>(std::min<double>(value, prop_info.absMax),
                             prop_info.absMin);
    prop.absValue = value;
    PGERROR(camera.SetProperty(&prop), "Failed to set property");
  }
}

void WriteRegister(Camera& camera, unsigned address, unsigned value) {
  PGERROR(camera.WriteRegister(address, value), "Failed to write register");
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
  PGERROR(camera.SetEmbeddedImageInfo(&info), "Failed to enable metadata");
}

}  // namespace flea3
