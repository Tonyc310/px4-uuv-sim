cmake_minimum_required(VERSION 3.22)
project(uuv_validation)

find_package(ament_cmake REQUIRED)
find_package(uuv_dynamics REQUIRED)

set(WARNINGS -Wall -Wextra -Wpedantic -Werror)

add_executable(simulate src/simulate.cpp)
target_compile_features(simulate PRIVATE cxx_std_17)
target_compile_options(simulate PRIVATE ${WARNINGS})
target_link_libraries(simulate PRIVATE uuv_dynamics::uuv_dynamics)

install(TARGETS simulate RUNTIME DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  # As in uuv_dynamics: neither copyright check, so cpplint runs here without its own.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS -legal/copyright)

  # The tests run the program just built.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(simulate_program test/test_simulate.py
    ENV SIMULATE=${CMAKE_CURRENT_BINARY_DIR}/simulate)
  ament_add_pytest_test(fossen_simulator test/test_fossen_simulator.py
    ENV SIMULATE=${CMAKE_CURRENT_BINARY_DIR}/simulate
    TIMEOUT 300)

  # Gazebo runs in its own transport partition: PX4's startup joins any Gazebo it finds in its
  # partition, and colcon runs the packages' tests side by side.
  find_package(ament_cmake_gtest REQUIRED)
  find_package(gz_sim_vendor REQUIRED)
  find_package(gz-sim REQUIRED)
  ament_add_gtest(test_gazebo_hydrodynamics test/test_gazebo_hydrodynamics.cpp
    ENV GZ_PARTITION=${PROJECT_NAME}_test
    TIMEOUT 120)
  target_compile_options(test_gazebo_hydrodynamics PRIVATE ${WARNINGS})
  target_compile_definitions(test_gazebo_hydrodynamics PRIVATE
    GAZEBO_HYDRODYNAMICS_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/worlds/gazebo_hydrodynamics.sdf")
  target_link_libraries(test_gazebo_hydrodynamics gz-sim::gz-sim uuv_dynamics::uuv_dynamics)
endif()

ament_package()
