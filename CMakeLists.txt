cmake_minimum_required(VERSION 3.8)
project(mps_map_gen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

include_directories(include)
include_directories(PRIVATE ${tf2_INCLUDE_DIRS} PRIVATE ${tf2_ros_INCLUDE_DIRS})

add_executable(mps_map_gen src/mps_map_gen.cpp src/mps.cpp)
target_include_directories(
  mps_map_gen PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                     $<INSTALL_INTERFACE:include>)
target_compile_features(mps_map_gen PUBLIC c_std_99 cxx_std_17)
target_link_libraries(mps_map_gen Eigen3::Eigen ${tf2_LIBRARIES})
ament_target_dependencies(
  mps_map_gen
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  map_msgs
  geometry_msgs)

install(TARGETS mps_map_gen DESTINATION lib/${PROJECT_NAME})

# Install include directory
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_package()
