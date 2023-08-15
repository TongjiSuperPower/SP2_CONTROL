cmake_minimum_required(VERSION 3.8)
project(chassis_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
# 添加所有用到的ROS2功能包
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  controller_interface
  realtime_tools
  pluginlib
  control_toolbox

  generate_parameter_library
  std_msgs
  geometry_msgs
)
foreach(Dependencies IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependencies} REQUIRED)  
endforeach()

generate_parameter_library(roller_wheel_controller_parameter
  config/roller_wheel_controller_parameter.yaml
)

add_library(${PROJECT_NAME} SHARED 
  src/chassis_base.cpp
  src/roller_wheel.cpp
)

ament_target_dependencies(${PROJECT_NAME} PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(${PROJECT_NAME} PUBLIC roller_wheel_controller_parameter)
pluginlib_export_plugin_description_file(controller_interface chassis_controllers_plugin.xml)

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

install(
  DIRECTORY include/
  DESTINATION include/
)
install(TARGETS ${PROJECT_NAME} roller_wheel_controller_parameter
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
