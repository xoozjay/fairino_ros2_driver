cmake_minimum_required(VERSION 3.8)
project(fr_robot_driver)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/zero_pose_collision_check.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES mock_components_plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
