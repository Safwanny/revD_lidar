cmake_minimum_required(VERSION 3.8)
project(revd_lidar_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_launch_test(
    test/test_nav2.py
    TIMEOUT 400
  )
  set_tests_properties(
    test_test_nav2.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=43;GZ_PARTITION=revd_lidar_nav2_test"
      RUN_SERIAL TRUE
  )
endif()

ament_package()
