cmake_minimum_required(VERSION 3.16)
project(forward_command_controller_traced LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

# The upstream tests target newer controller_interface APIs than some Humble installations.
# Force tests off for this traced fork so runtime builds are not blocked on test API drift.
set(FORWARD_COMMAND_CONTROLLER_TRACED_BUILD_TESTS OFF CACHE BOOL "Build traced controller tests" FORCE)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  generate_parameter_library
  hardware_interface
  marvin_trace_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  runtime_interfaces
  sensor_msgs
  std_msgs
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(OpenSSL REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  forward_command_controller_traced_parameters
  src/forward_command_controller_parameters.yaml
)
generate_parameter_library(
  multi_interface_forward_command_controller_traced_parameters
  src/multi_interface_forward_command_controller_parameters.yaml
)

add_library(forward_command_controller_traced SHARED
  src/experimental_joint_hold_controller.cpp
  src/supervised_command_controller.cpp
  src/forward_controllers_base.cpp
  src/forward_command_controller.cpp
  src/multi_interface_forward_command_controller.cpp
)
target_compile_features(forward_command_controller_traced PUBLIC cxx_std_17)
target_include_directories(forward_command_controller_traced PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(forward_command_controller_traced PUBLIC
  OpenSSL::Crypto
  forward_command_controller_traced_parameters
  multi_interface_forward_command_controller_traced_parameters
)
ament_target_dependencies(forward_command_controller_traced PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(forward_command_controller_traced PRIVATE "FORWARD_COMMAND_CONTROLLER_BUILDING_DLL")
pluginlib_export_plugin_description_file(controller_interface forward_command_plugin.xml)

# Focused tests use Jazzy's controller init API; the upstream suite above remains
# independently gated for its newer API. No real hardware is opened.
if(BUILD_TESTING)
  add_executable(test_diagnostic_execution test/test_diagnostic_execution.cpp)
  target_include_directories(test_diagnostic_execution PRIVATE include)
  target_compile_features(test_diagnostic_execution PRIVATE cxx_std_17)
  target_compile_options(test_diagnostic_execution PRIVATE -UNDEBUG)
  add_test(NAME test_diagnostic_execution COMMAND test_diagnostic_execution)
  add_executable(test_joint_sample_validity test/test_joint_sample_validity.cpp)
  target_include_directories(test_joint_sample_validity PRIVATE include)
  target_compile_features(test_joint_sample_validity PRIVATE cxx_std_17)
  target_compile_options(test_joint_sample_validity PRIVATE -UNDEBUG)
  add_test(NAME test_joint_sample_validity COMMAND test_joint_sample_validity)
endif()
if(BUILD_TESTING AND "$ENV{ROS_DISTRO}" STREQUAL "jazzy")
  find_package(ament_cmake_gtest REQUIRED)
  add_executable(check_joint_chirp test/check_joint_chirp.cpp)
  target_include_directories(check_joint_chirp PRIVATE include)
  target_compile_features(check_joint_chirp PRIVATE cxx_std_17)
  target_compile_options(check_joint_chirp PRIVATE -UNDEBUG)
  add_test(NAME check_joint_chirp COMMAND check_joint_chirp)
  ament_add_gtest(test_tracing_modes test/test_tracing_modes.cpp)
  target_link_libraries(test_tracing_modes forward_command_controller_traced)
  ament_add_gtest(test_joint_trajectory test/test_joint_trajectory.cpp)
  target_link_libraries(test_joint_trajectory forward_command_controller_traced)
  ament_add_gtest(test_joint_playback test/test_joint_playback.cpp)
  target_link_libraries(test_joint_playback forward_command_controller_traced)
  ament_add_gtest(test_joint_admission test/test_joint_admission.cpp)
  target_link_libraries(test_joint_admission forward_command_controller_traced)
  ament_target_dependencies(test_joint_admission runtime_interfaces)
endif()

if(BUILD_TESTING AND FORWARD_COMMAND_CONTROLLER_TRACED_BUILD_TESTS)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_load_forward_command_controller
    test/test_load_forward_command_controller.cpp
  )
  target_link_libraries(test_load_forward_command_controller
    forward_command_controller_traced
  )
  ament_target_dependencies(test_load_forward_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_forward_command_controller
    test/test_forward_command_controller.cpp
  )
  target_link_libraries(test_forward_command_controller
    forward_command_controller_traced
  )

  ament_add_gmock(test_load_multi_interface_forward_command_controller
    test/test_load_multi_interface_forward_command_controller.cpp
  )
  target_link_libraries(test_load_multi_interface_forward_command_controller
    forward_command_controller_traced
  )
  ament_target_dependencies(test_load_multi_interface_forward_command_controller
    controller_manager
    hardware_interface
    ros2_control_test_assets
  )

  ament_add_gmock(test_multi_interface_forward_command_controller
    test/test_multi_interface_forward_command_controller.cpp
  )
  target_link_libraries(test_multi_interface_forward_command_controller
    forward_command_controller_traced
  )
endif()

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS
    forward_command_controller_traced
    forward_command_controller_traced_parameters
    multi_interface_forward_command_controller_traced_parameters
  EXPORT export_forward_command_controller_traced
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

ament_export_targets(export_forward_command_controller_traced HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_dependencies(OpenSSL)
ament_package()
