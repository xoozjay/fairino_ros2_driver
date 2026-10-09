// Which robot is it? The controller reports a model string in GetSoftwareVersion (first string,
// e.g. "FR5-V1-002(V6.0)"); this maps it to the fr_description model of the same robot.
//
// Source: the controller model table of the official manual
// (references/fr_documentation, CobotsManual/system.rst, "机器人型号配置"). Verified on a
// controller only for FR5 V6.0 (the simulator); the other rows are matched by name. ❓
//
// Not mapped on purpose, because fr_description has no matching kinematics: the V4.0 / V5.0
// generations, "V6.0 Mirror", ART3/ART5, FR10YD, FR3-C. Those are reported as unknown.
#pragma once

#include <string_view>

namespace fr_client_library
{
struct ModelEntry
{
  const char * controller_model;   // what the controller reports
  const char * description_model;  // fr_description model directory name
};

inline constexpr ModelEntry kKnownModels[] = {
  {"FR3-V1-001(V6.0)", "fairino3_v6"},
  {"FR5-V1-002(V6.0)", "fairino5_v6"},
  {"FR10-V1-001(V6.0)", "fairino10_v6"},
  {"FR16-V1-001(V6.0)", "fairino16_v6"},
  {"FR20-V1-001(V6.0)", "fairino20_v6"},
  {"FR30-V1-001(V6.0)", "fairino30_v6"},
  {"FRCustom(7)-V1-001(FR3-WML)", "fr3wml"},
  {"FRCustom(7)-V1-001(FR3-WMS)", "fr3wms"},
  {"FRCustom(8)-V1-001(FR5WM)", "fr5wm"},
  {"FRCustom(9)-V1-001(FR3MT)", "fairino3_mt_v6"},
};

// nullptr if the model is not one this driver has kinematics for.
constexpr const char * descriptionModelFor(std::string_view controller_model)
{
  for (const auto & e : kKnownModels) {
    if (controller_model == e.controller_model) {
      return e.description_model;
    }
  }
  return nullptr;
}

}  // namespace fr_client_library
