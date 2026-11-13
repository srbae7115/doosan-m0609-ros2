cmake_minimum_required(VERSION 3.16)
project(m0609_mirror CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dsr_common2 REQUIRED)

add_executable(drfl_mirror src/drfl_mirror.cpp)
target_compile_definitions(drfl_mirror PRIVATE DRCF_VERSION=2)  # controller GF02.x
target_link_libraries(drfl_mirror dsr_common2::drfl)
ament_target_dependencies(drfl_mirror rclcpp sensor_msgs std_msgs)

install(TARGETS drfl_mirror DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/scene_markers.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
