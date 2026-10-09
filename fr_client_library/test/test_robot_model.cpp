#include <gtest/gtest.h>

#include <set>
#include <string>

#include "fr_client_library/robot_model.hpp"

using namespace fr_client_library;

TEST(RobotModel, TheSimulatorsModelIsFr5V6)
{
  EXPECT_STREQ(descriptionModelFor("FR5-V1-002(V6.0)"), "fairino5_v6");  // as reported by the simulator
}

TEST(RobotModel, EveryV6ModelMapsToItsDescription)
{
  EXPECT_STREQ(descriptionModelFor("FR3-V1-001(V6.0)"), "fairino3_v6");
  EXPECT_STREQ(descriptionModelFor("FR10-V1-001(V6.0)"), "fairino10_v6");
  EXPECT_STREQ(descriptionModelFor("FR16-V1-001(V6.0)"), "fairino16_v6");
  EXPECT_STREQ(descriptionModelFor("FR20-V1-001(V6.0)"), "fairino20_v6");
  EXPECT_STREQ(descriptionModelFor("FR30-V1-001(V6.0)"), "fairino30_v6");
  EXPECT_STREQ(descriptionModelFor("FRCustom(7)-V1-001(FR3-WML)"), "fr3wml");
  EXPECT_STREQ(descriptionModelFor("FRCustom(7)-V1-001(FR3-WMS)"), "fr3wms");
  EXPECT_STREQ(descriptionModelFor("FRCustom(8)-V1-001(FR5WM)"), "fr5wm");
  EXPECT_STREQ(descriptionModelFor("FRCustom(9)-V1-001(FR3MT)"), "fairino3_mt_v6");
}

TEST(RobotModel, GenerationsWithoutKinematicsAreUnknownNotGuessed)
{
  for (const char * m : {"FR5-V1-001(V5.0)", "FR5-V1-000", "FR3-V1-000(V5.0)", "FR20-V1-000(V5.0)",
      "FR3-V1-002(V6.0 Mirror)", "ART3-V1-000", "ART5-V1-000", "FRCustom(9)-V1-001(FR10YD)",
      "FRCustom(9)-V1-001(FR3-C)", "", "FR5", "fr5-v1-002(v6.0)"})
  {
    EXPECT_EQ(descriptionModelFor(m), nullptr) << m;
  }
}

TEST(RobotModel, TableHasNoDuplicates)
{
  std::set<std::string> controller, description;
  for (const auto & e : kKnownModels) {
    EXPECT_TRUE(controller.insert(e.controller_model).second) << e.controller_model;
    EXPECT_TRUE(description.insert(e.description_model).second) << e.description_model;
  }
}
