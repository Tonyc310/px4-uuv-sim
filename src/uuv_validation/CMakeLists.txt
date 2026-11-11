cmake_minimum_required(VERSION 3.22)
project(uuv_validation)

find_package(ament_cmake REQUIRED)
find_package(uuv_dynamics REQUIRED)

add_executable(simulate src/simulate.cpp)
target_compile_features(simulate PRIVATE cxx_std_17)
target_compile_options(simulate PRIVATE -Wall -Wextra -Wpedantic -Werror)
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
endif()

ament_package()
