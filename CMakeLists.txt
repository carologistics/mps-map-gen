cmake_minimum_required(VERSION 3.8)
project(mps_map_gen)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(map_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(ProtobufComm REQUIRED)
find_package(rcll_protobuf_cpp REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(include)
include_directories(PRIVATE ${tf2_INCLUDE_DIRS} PRIVATE ${tf2_ros_INCLUDE_DIRS})
add_library(mps SHARED src/mps.cpp)
add_library(wait_pos SHARED src/waiting_positions.cpp)
add_library(refbox_connector SHARED src/refbox_connector.cpp)

add_executable(${PROJECT_NAME} src/mps_map_gen.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen ${tf2_LIBRARIES} mps refbox_connector wait_pos)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME} PRIVATE ${Boost_INCLUDE_DIRS})
ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  map_msgs
  geometry_msgs)

target_include_directories(refbox_connector PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
target_link_libraries(refbox_connector Boost::boost Boost::system ProtobufComm::protobuf_comm mps)
ament_target_dependencies(refbox_connector Protobuf ProtobufComm rcll_protobuf_cpp)

target_link_libraries(mps Eigen3::Eigen ${tf2_LIBRARIES})
target_include_directories(mps PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(mps tf2 tf2_msgs tf2_geometry_msgs)

target_link_libraries(wait_pos Eigen3::Eigen ${tf2_LIBRARIES})
target_include_directories(wait_pos PUBLIC ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(
  wait_pos
  rclcpp
  nav_msgs
  tf2
  tf2_ros
  tf2_msgs
  tf2_geometry_msgs
  map_msgs
  ament_index_cpp
  rcll_protobuf_cpp
  geometry_msgs)

install(TARGETS mps_map_gen mps wait_pos refbox_connector DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install include directory
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line when this package is in a git repo and when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_libraries(wait_pos refbox_connector mps)
ament_export_include_directories(include)
ament_package()
