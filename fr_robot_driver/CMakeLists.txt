cmake_minimum_required(VERSION 3.8)
project(fr_robot_driver)

find_package(ament_cmake REQUIRED)
find_package(fr_client_library REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_srvs REQUIRED)

add_library(fr_robot_hardware_interface SHARED src/fr_robot_hardware_interface.cpp)
target_include_directories(fr_robot_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(fr_robot_hardware_interface PUBLIC cxx_std_17)
target_compile_options(fr_robot_hardware_interface PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_link_libraries(fr_robot_hardware_interface PUBLIC
  fr_client_library::fr_client_library
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
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

  ament_add_gtest(test_reset_policy test/test_reset_policy.cpp)
  target_link_libraries(test_reset_policy fr_robot_hardware_interface)

  # needs the installed plugin (pluginlib) and the installed xacro: run via colcon test
  ament_add_gtest(test_hardware_interface test/test_hardware_interface.cpp TIMEOUT 180
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_hardware_interface fr_robot_hardware_interface
    ament_index_cpp::ament_index_cpp ${lifecycle_msgs_TARGETS})
endif()

ament_export_libraries(fr_robot_hardware_interface)
ament_package()
