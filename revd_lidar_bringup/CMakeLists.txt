cmake_minimum_required(VERSION 3.8)
project(revd_lidar_bringup)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/generate_sim_sdf.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_test(
    NAME verify_mecanum_wheel_contact
    COMMAND
      ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_mecanum_wheel_contact.py
      ${CMAKE_CURRENT_SOURCE_DIR}
  )

  add_launch_test(
    test/test_sensor_topics.py
    TIMEOUT 180
  )
  set_tests_properties(
    test_test_sensor_topics.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=41;GZ_PARTITION=revd_lidar_sensor_test"
      RUN_SERIAL TRUE
  )

  add_launch_test(
    test/test_safety_chain.py
    TIMEOUT 600
  )
  set_tests_properties(
    test_test_safety_chain.py
    PROPERTIES
      ENVIRONMENT
        "ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros_logs;ROS_DOMAIN_ID=42;GZ_PARTITION=revd_lidar_safety_test"
      RUN_SERIAL TRUE
  )
endif()

ament_package()
