cmake_minimum_required(VERSION 3.8)
project(canopen_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    # add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  canopen_402_driver
  canopen_master_driver
  canopen_core
  canopen_proxy_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
)

find_package(ros2_control_test_assets)
find_package(ament_cmake_gmock REQUIRED)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  canopen_ros2_control
  SHARED
  src/canopen_system.cpp
  src/cia402_system.cpp
  src/robot_system.cpp
)



target_compile_features(canopen_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_options(canopen_ros2_control PUBLIC -fPIC -Wl,--no-undefined)

target_include_directories(canopen_ros2_control PUBLIC include)
ament_target_dependencies(canopen_ros2_control ${THIS_PACKAGE_INCLUDE_DEPENDS})

# prevent pluginlib from using boost
target_compile_definitions(canopen_ros2_control PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(hardware_interface canopen_ros2_control.xml)

install(
  TARGETS
  canopen_ros2_control
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  #find_package(ament_cmake_gmock REQUIRED)
  #find_package(ros2_control_test_assets REQUIRED)

  #ament_add_gmock(test_canopen_system test/test_canopen_system.cpp)
  #target_include_directories(test_canopen_system PRIVATE include)
  #ament_target_dependencies(
    #test_canopen_system ${THIS_PACKAGE_INCLUDE_DEPENDS} ros2_control_test_assets)
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  canopen_ros2_control
)
ament_export_dependencies(
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

ament_package()
