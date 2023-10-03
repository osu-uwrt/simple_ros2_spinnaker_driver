cmake_minimum_required(VERSION 3.8)
project(simple_ros2_spinnaker_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wpedantic)
endif()

#
# configure library path for Spinnaker
#
set(SPINNAKER_LIBRARY_PATH "/opt/spinnaker")

#
# find dependencies
#
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)

#
# configure executables
#
file(GLOB sources src/simple_ros2_spinnaker_driver/*.cpp)
add_executable(spinnaker_node
  ${sources})

target_link_directories(spinnaker_node PUBLIC 
  ${SPINNAKER_LIBRARY_PATH}/lib 
)

target_include_directories(spinnaker_node PUBLIC
  #spinnaker library headers
  ${SPINNAKER_LIBRARY_PATH}/include
  #package headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(spinnaker_node Spinnaker)

ament_target_dependencies(spinnaker_node
  rclcpp
  sensor_msgs
  image_transport)

#
# install package executables and directories
#
install(TARGETS spinnaker_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})  

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
