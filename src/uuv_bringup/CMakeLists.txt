cmake_minimum_required(VERSION 3.22)
project(uuv_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  # ROS 2's linters that package.xml lists. Not the copyright check: the portfolio's files carry no
  # author headers.
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(underwater_example test/test_underwater_example.py TIMEOUT 120)
endif()

ament_package()
