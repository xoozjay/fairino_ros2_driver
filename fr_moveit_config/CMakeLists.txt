cmake_minimum_required(VERSION 3.8)
project(fr_moveit_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(srdfdom REQUIRED)
find_package(urdf REQUIRED)

install(DIRECTORY config srdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_srdf test/test_srdf.cpp)
  target_link_libraries(test_srdf
    ament_index_cpp::ament_index_cpp
    moveit_core::moveit_robot_model
    srdfdom::srdfdom
    urdf::urdf
  )
endif()

ament_package()
