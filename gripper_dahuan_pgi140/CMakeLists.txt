cmake_minimum_required(VERSION 3.8)
project(gripper_dahuan_pgi140)

find_package(ament_cmake REQUIRED)

install(DIRECTORY collision urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
