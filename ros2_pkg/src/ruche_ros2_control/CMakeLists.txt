
cmake_minimum_required(VERSION 3.16)
project(ruche_ros2_control LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  backward_ros
  ament_cmake
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  PkgConfig
)
# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. ")
endif()

# find dependencies
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(${Dependency} REQUIRED)
endforeach()

# find dep with pkg-config
pkg_check_modules(SDBUS REQUIRED sdbus-c++)

## COMPILE
add_library(
  ruche_ros2_control
  SHARED
  hardware/elegoo_bt2serial.cpp
  )
  target_compile_features(ruche_ros2_control PUBLIC cxx_std_17 ${SDBUS_CFLAGS_OTHER})
  target_include_directories(ruche_ros2_control PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/ruche_ros2_control>
  )
  target_link_libraries(ruche_ros2_control PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  bluetooth
  ${SDBUS_LIBRARIES}
  )
  
  # Export hardware plugins
  pluginlib_export_plugin_description_file(hardware_interface elegoo_bt2serial.xml) # plugin xml file
  
  # INSTALL
  install(
    DIRECTORY hardware/include/.
    DESTINATION include/${PROJECT_NAME}
    )
    
    install(
      DIRECTORY description/urdf bringup/launch bringup/config
      DESTINATION share/${PROJECT_NAME}
      )
      
      install(
        DIRECTORY description/utils/ description/ros2_control/
  DESTINATION share/${PROJECT_NAME}/urdf
  )
  
install(
  DIRECTORY description/rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
  )
  install(TARGETS ruche_ros2_control
  EXPORT export_ruche_ros2_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  )
  
## EXPORTS
ament_export_targets(export_ruche_ros2_control HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
