cmake_minimum_required(VERSION 3.8)
project(fr_robot_driver)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(fr_client_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(fr_robot_hardware_interface SHARED src/fr_robot_hardware_interface.cpp)
target_include_directories(fr_robot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fr_robot_hardware_interface PUBLIC cxx_std_17)
target_compile_options(fr_robot_hardware_interface PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(fr_robot_hardware_interface PUBLIC
  ${control_msgs_TARGETS}
  ${diagnostic_msgs_TARGETS}
  fr_client_library::fr_client_library
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
  ${std_srvs_TARGETS})
pluginlib_export_plugin_description_file(hardware_interface fr_robot_driver.xml)

install(TARGETS fr_robot_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/zero_pose_collision_check.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  find_package(lifecycle_msgs REQUIRED)

  ament_add_gtest(test_arm_gate test/test_arm_gate.cpp)
  target_link_libraries(test_arm_gate fr_robot_hardware_interface)
  target_include_directories(test_arm_gate PRIVATE include)
  target_compile_features(test_arm_gate PRIVATE cxx_std_17)
  target_compile_options(test_arm_gate PRIVATE -Wall -Wextra -Wpedantic -Werror)

  ament_add_gtest(test_reset_policy test/test_reset_policy.cpp)
  target_link_libraries(test_reset_policy fr_robot_hardware_interface)

  ament_add_gtest(test_status_report test/test_status_report.cpp)
  target_link_libraries(test_status_report fr_robot_hardware_interface)

  ament_add_gtest(test_gripper_goals test/test_gripper_goals.cpp)
  target_link_libraries(test_gripper_goals fr_robot_hardware_interface)

  ament_add_gtest(test_gripper_motion test/test_gripper_motion.cpp)
  target_include_directories(test_gripper_motion PRIVATE include)
  target_compile_features(test_gripper_motion PRIVATE cxx_std_17)
  target_compile_options(test_gripper_motion PRIVATE -Wall -Wextra -Wpedantic -Werror)

  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_include_directories(test_kinematics PRIVATE include)

  # needs the installed plugin (pluginlib) and the installed xacro: run via colcon test
  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp TIMEOUT 180
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_hardware_interface fr_robot_hardware_interface
    ament_index_cpp::ament_index_cpp ${lifecycle_msgs_TARGETS})

  # a controller manager loop with joint_trajectory_controller and the installed controllers.yaml
  find_package(controller_manager REQUIRED)
  ament_add_gtest(test_trajectory_pause test/test_trajectory_pause.cpp TIMEOUT 120
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_trajectory_pause fr_robot_hardware_interface
    ament_index_cpp::ament_index_cpp controller_manager::controller_manager)
endif()

ament_export_libraries(fr_robot_hardware_interface)
ament_package()
