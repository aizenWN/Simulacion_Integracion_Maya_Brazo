cmake_minimum_required(VERSION 3.8)
project(maya_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
install(
    DIRECTORY launch config world
    DESTINATION share/${PROJECT_NAME}/
)
install(PROGRAMS
    scripts/scan_frame_relay.py
    scripts/scan_merge.py
    scripts/sim_covariance_relay.py
    scripts/square_test.py
    DESTINATION lib/${PROJECT_NAME}
)
ament_package()
