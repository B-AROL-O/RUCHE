
cmake_minimum_required(VERSION 3.8)
project(ruche_ros2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  elegoo_bt2serial
  SHARED
  hardware/elegoo_bt2serial.cpp
)
target_compile_features(elegoo_bt2serial PUBLIC cxx_std_17)
target_include_directories(elegoo_bt2serial PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/elegoo_bt2serial>
)
ament_target_dependencies(
  elegoo_bt2serial PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(elegoo_bt2serial PUBLIC serial)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface elegoo_bt2serial.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  DIRECTORY bringup/launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY bringup/config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY description/urdf/ description/ros2_control/ description/utils/
  DESTINATION share/${PROJECT_NAME}/description
)

install(
  DIRECTORY description/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(TARGETS elegoo_bt2serial
  EXPORT export_elegoo_bt2serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

## EXPORTS
ament_export_targets(export_elegoo_bt2serial HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
