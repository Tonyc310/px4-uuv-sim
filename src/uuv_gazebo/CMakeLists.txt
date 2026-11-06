cmake_minimum_required(VERSION 3.22)
project(uuv_gazebo)

find_package(ament_cmake REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED COMPONENTS register)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(uuv_dynamics REQUIRED)

set(WARNINGS -Wall -Wextra -Wpedantic -Werror)

# Gazebo loads the plugin by its file name from GZ_SIM_SYSTEM_PLUGIN_PATH, which the hook below sets.
set(PLUGIN ${PROJECT_NAME}_fossen_dynamics)
add_library(${PLUGIN} SHARED src/fossen_dynamics.cpp)
target_compile_features(${PLUGIN} PRIVATE cxx_std_17)
target_compile_options(${PLUGIN} PRIVATE ${WARNINGS})
target_link_libraries(${PLUGIN} PRIVATE
  gz-plugin::register gz-sim::gz-sim uuv_dynamics::uuv_dynamics)

install(TARGETS ${PLUGIN} LIBRARY DESTINATION lib)
install(DIRECTORY models worlds DESTINATION share/${PROJECT_NAME})
ament_environment_hooks(hooks/${PROJECT_NAME}.dsv.in)

if(BUILD_TESTING)
  # As in uuv_dynamics: neither copyright check, so cpplint runs here without its own.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS -legal/copyright)

  # Gazebo finds the plugin and the model in the build and source trees. The test's Gazebo gets its
  # own transport partition: PX4's startup joins any Gazebo it finds running in its partition, and
  # colcon runs the packages' tests side by side.
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_fossen_dynamics test/test_fossen_dynamics.cpp
    ENV GZ_PARTITION=${PROJECT_NAME}_test
    APPEND_ENV
      GZ_SIM_SYSTEM_PLUGIN_PATH=${CMAKE_CURRENT_BINARY_DIR}
      GZ_SIM_RESOURCE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/models
    TIMEOUT 120)
  target_compile_options(test_fossen_dynamics PRIVATE ${WARNINGS})
  target_compile_definitions(test_fossen_dynamics PRIVATE
    WATER_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/worlds/water.sdf")
  target_link_libraries(test_fossen_dynamics gz-sim::gz-sim uuv_dynamics::uuv_dynamics)
  add_dependencies(test_fossen_dynamics ${PLUGIN})
endif()

ament_package()
