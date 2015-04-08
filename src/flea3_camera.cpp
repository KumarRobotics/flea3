#include "flea3/flea3_camera.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

namespace flea3 {

using namespace FlyCapture2;

Flea3Camera::Flea3Camera(const std::string& serial) : serial_(serial) {
  Connect();
}

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
    capturing_ = false;
  }
}

void Flea3Camera::StopCapture() {
  if (camera_.IsConnected() && capturing_) {
    PGERROR(camera_.StopCapture(), "Failed to stop capture");
    capturing_ = false;
  }
}

void Flea3Camera::Configure(Config& config) {}

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

bool Flea3Camera::GrabImage(sensor_msgs::Image& image_msg,
                            sensor_msgs::CameraInfo& cinfo_msg) {
  if (!(camera_.IsConnected() && capturing_)) return false;

  Image image;
  PGERROR(camera_.RetrieveBuffer(&image), "Failed to retrieve buffer");
  //  auto metadata = image.GetMetadata();

  // TODO: Change this to use_ros_time?
  if (true) {
    auto time = image.GetTimeStamp();
    image_msg.header.stamp.sec = time.seconds;
    image_msg.header.stamp.nsec = 1000 * time.microSeconds;
    cinfo_msg.header.stamp = image_msg.header.stamp;
  }

  // Set image encodings
  const auto bayer_format = image.GetBayerTileFormat();
  const auto bits_per_pixel = image.GetBitsPerPixel();
  std::string encoding;
  if (camera_info_.isColorCamera && bayer_format != NONE) {
    encoding = BayerFormatToEncoding(bayer_format);
  } else {
    encoding = PixelFormatToEncoding(bits_per_pixel);
  }
  return sensor_msgs::fillImage(image_msg, encoding, image.GetRows(),
                                image.GetCols(), image.GetStride(),
                                image.GetData());
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

}  // namespace flea3
