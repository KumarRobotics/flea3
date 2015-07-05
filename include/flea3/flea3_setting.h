#ifndef FLEA3_SETTING_H_
#define FLEA3_SETTING_H_

#include <flycapture/FlyCapture2.h>

#define PGERROR(err, msg) flea3::HandleError(err, msg, __func__)

namespace flea3 {

using namespace FlyCapture2;

void HandleError(const Error& error, const std::string& message = "",
                 const std::string& func_name = "");

void PrintPropertyInfo(const PropertyInfo& prop_info,
                       const std::string& prop_name);
void PrintProperty(const Property& prop, const std::string& prop_name);

std::string BayerFormatToEncoding(const BayerTileFormat& bayer_format);
std::string PixelFormatToEncoding(unsigned bits_per_pixel);

PropertyInfo GetPropertyInfo(Camera& camera, const PropertyType& prop_type);
Property GetProperty(Camera& camera, const PropertyType& prop_type);

void SetProperty(Camera& camera, const PropertyType& prop_type, bool& auto_on,
                 double& value);
void SetProperty(Camera& camera, const PropertyType& prop_type, double& value);

unsigned ReadRegister(Camera& camera, unsigned address);
void WriteRegister(Camera& camera, unsigned address, unsigned value);

void EnableMetadata(Camera& camera);

}  // namespace flea3

#endif  // FLEA3_SETTING_H_
