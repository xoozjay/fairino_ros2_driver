cmake_minimum_required(VERSION 3.8)
project(fr_client_library)

find_package(ament_cmake REQUIRED)

add_library(${PROJECT_NAME}
  src/frame_parser.cpp
  src/gripper_sender.cpp
  src/http_connection.cpp
  src/instant_command.cpp
  src/robot_api.cpp
  src/servo_sender.cpp
  src/software_version.cpp
  src/state_receiver.cpp
  src/tcp_socket.cpp
  src/xmlrpc_codec.cpp)
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic -Werror)
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(fr_probe src/fr_probe.cpp)
target_link_libraries(fr_probe ${PROJECT_NAME})
target_compile_options(fr_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
install(TARGETS fr_probe DESTINATION lib/${PROJECT_NAME})

add_executable(fr_servo_bench src/fr_servo_bench.cpp)
target_link_libraries(fr_servo_bench ${PROJECT_NAME})
target_compile_options(fr_servo_bench PRIVATE -Wall -Wextra -Wpedantic -Werror)
install(TARGETS fr_servo_bench DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  foreach(t robot_state software_version frame_parser state_receiver instant_command
      xmlrpc_codec http_connection robot_api servo_sender spsc_queue fault_state robot_model)
    ament_add_gtest(test_${t} test/test_${t}.cpp)
    target_link_libraries(test_${t} ${PROJECT_NAME})
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
