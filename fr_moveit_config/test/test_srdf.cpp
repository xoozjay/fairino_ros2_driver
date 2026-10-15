#include <gtest/gtest.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/robot_model/robot_model.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace
{

struct FileCloser
{
  void operator()(FILE* f) const
  {
    if (f) {
      pclose(f);
    }
  }
};

std::string run_cmd(const std::string& cmd)
{
  std::array<char, 4096> buffer;
  std::string result;
  std::unique_ptr<FILE, FileCloser> pipe(popen(cmd.c_str(), "r"));
  if (!pipe) {
    throw std::runtime_error("popen() failed: " + cmd);
  }
  while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
    result += buffer.data();
  }
  return result;
}

const std::vector<std::string> kModels = {
  "fairino3_v6", "fairino3_mt_v6", "fairino5_v6", "fairino10_v6",
  "fairino16_v6", "fairino20_v6", "fairino30_v6", "fr3wml", "fr3wms", "fr5wm"
};

}  // namespace

class SrdfModelTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(SrdfModelTest, LoadsUrdfAndSrdfWithoutGripper)
{
  const std::string model_name = GetParam();
  const std::string driver_share = ament_index_cpp::get_package_share_directory("fr_robot_driver");
  const std::string moveit_share = ament_index_cpp::get_package_share_directory("fr_moveit_config");

  const std::string urdf_cmd = "xacro " + driver_share + "/urdf/fr_controlled.urdf.xacro model:=" + model_name;
  const std::string srdf_cmd = "xacro " + moveit_share + "/srdf/fr.srdf.xacro model:=" + model_name;

  const std::string urdf_xml = run_cmd(urdf_cmd);
  ASSERT_FALSE(urdf_xml.empty());
  const std::string srdf_xml = run_cmd(srdf_cmd);
  ASSERT_FALSE(srdf_xml.empty());

  auto urdf_model = std::make_shared<urdf::Model>();
  ASSERT_TRUE(urdf_model->initString(urdf_xml));

  auto srdf_model = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf_model->initString(*urdf_model, srdf_xml));

  moveit::core::RobotModel robot_model(urdf_model, srdf_model);
  EXPECT_TRUE(robot_model.hasJointModelGroup("arm"));
  const auto* arm_group = robot_model.getJointModelGroup("arm");
  ASSERT_NE(arm_group, nullptr);
  EXPECT_EQ(arm_group->getActiveJointModelNames().size(), 6u);
}

TEST_P(SrdfModelTest, LoadsUrdfAndSrdfWithGripperAndPrefix)
{
  const std::string model_name = GetParam();
  const std::string driver_share = ament_index_cpp::get_package_share_directory("fr_robot_driver");
  const std::string moveit_share = ament_index_cpp::get_package_share_directory("fr_moveit_config");

  const std::string urdf_cmd = "xacro " + driver_share + "/urdf/fr_controlled.urdf.xacro model:=" + model_name +
                               " gripper:=pgi140 tf_prefix:=test_";
  const std::string srdf_cmd = "xacro " + moveit_share + "/srdf/fr.srdf.xacro model:=" + model_name +
                               " gripper:=pgi140 tf_prefix:=test_";

  const std::string urdf_xml = run_cmd(urdf_cmd);
  ASSERT_FALSE(urdf_xml.empty());
  const std::string srdf_xml = run_cmd(srdf_cmd);
  ASSERT_FALSE(srdf_xml.empty());

  auto urdf_model = std::make_shared<urdf::Model>();
  ASSERT_TRUE(urdf_model->initString(urdf_xml));

  auto srdf_model = std::make_shared<srdf::Model>();
  ASSERT_TRUE(srdf_model->initString(*urdf_model, srdf_xml));

  moveit::core::RobotModel robot_model(urdf_model, srdf_model);
  EXPECT_TRUE(robot_model.hasJointModelGroup("test_arm"));
  EXPECT_TRUE(robot_model.hasJointModelGroup("test_gripper"));
  EXPECT_TRUE(robot_model.hasEndEffector("test_gripper"));

  // Verify finger collision disable rule is active
  EXPECT_TRUE(srdf_xml.find("test_left_finger_link") != std::string::npos);
  EXPECT_TRUE(srdf_xml.find("test_right_finger_link") != std::string::npos);
}

INSTANTIATE_TEST_SUITE_P(
  AllModels,
  SrdfModelTest,
  ::testing::ValuesIn(kModels),
  [](const ::testing::TestParamInfo<std::string>& info) {
    return info.param;
  }
);
