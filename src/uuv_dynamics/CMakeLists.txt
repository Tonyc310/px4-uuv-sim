cmake_minimum_required(VERSION 3.22)
project(uuv_dynamics)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Only on this package's own targets: gtest's sources, built alongside, aren't warning-free.
set(WARNINGS -Wall -Wextra -Wpedantic -Werror)

add_library(${PROJECT_NAME} SHARED
  src/dynamics.cpp src/frames.cpp src/motion.cpp src/thrusters.cpp src/vehicle.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME} PRIVATE ${WARNINGS})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  # ROS 2's linters that package.xml lists, without either copyright check: the portfolio's files
  # carry no author headers. ament_copyright isn't listed, and cpplint's own check is filtered out,
  # so cpplint runs here instead of through ament_lint_auto.
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_cpplint)
  ament_lint_auto_find_test_dependencies()
  ament_cpplint(FILTERS -legal/copyright)

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_frames test/test_frames.cpp)
  target_compile_options(test_frames PRIVATE ${WARNINGS})
  target_link_libraries(test_frames ${PROJECT_NAME})
  ament_add_gtest(test_dynamics test/test_dynamics.cpp)
  target_compile_options(test_dynamics PRIVATE ${WARNINGS})
  target_link_libraries(test_dynamics ${PROJECT_NAME})
  ament_add_gtest(test_motion test/test_motion.cpp)
  target_compile_options(test_motion PRIVATE ${WARNINGS})
  target_link_libraries(test_motion ${PROJECT_NAME})
  ament_add_gtest(test_thrusters test/test_thrusters.cpp)
  target_compile_options(test_thrusters PRIVATE ${WARNINGS})
  target_link_libraries(test_thrusters ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
