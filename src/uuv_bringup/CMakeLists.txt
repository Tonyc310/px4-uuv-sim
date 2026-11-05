cmake_minimum_required(VERSION 3.22)
project(uuv_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/px4_sitl DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # ROS 2's linters that package.xml lists. Not the copyright check: the portfolio's files carry no
  # author headers.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(underwater_example test/test_underwater_example.py TIMEOUT 120)
  ament_add_pytest_test(px4_sitl test/test_px4_sitl.py TIMEOUT 180)
  ament_add_pytest_test(ros2_bridge test/test_ros2_bridge.py TIMEOUT 180)
endif()

ament_package()
