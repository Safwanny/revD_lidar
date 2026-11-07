cmake_minimum_required(VERSION 3.8)
project(revd_lidar_safety)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(governor STATIC src/governor.cpp)
target_include_directories(governor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
set_target_properties(governor PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_compile_features(governor PUBLIC cxx_std_17)

add_library(safety_governor_component SHARED src/safety_governor_node.cpp)
target_link_libraries(safety_governor_component
  governor
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${std_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_components::component
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  tf2_ros::tf2_ros)
rclcpp_components_register_node(safety_governor_component
  PLUGIN "revd_lidar_safety::SafetyGovernor"
  EXECUTABLE safety_governor)

install(TARGETS safety_governor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_governor test/test_governor.cpp)
  target_link_libraries(test_governor governor)
  ament_add_gtest(test_parity test/test_parity.cpp)
  target_link_libraries(test_parity governor)
  target_compile_definitions(test_parity PRIVATE
    PARITY_CASES="${CMAKE_CURRENT_SOURCE_DIR}/test/parity_cases.txt")
endif()

ament_package()
