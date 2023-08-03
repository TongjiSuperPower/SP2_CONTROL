cmake_minimum_required(VERSION 3.8)
project(sp2_hw)

# 为了一些特性所以使用C++20标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE debug)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  yaml-cpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# YAML
find_package(generate_parameter_library REQUIRED)
generate_parameter_library(
  actuator_coefficient_lib # cmake target name for the parameter library
  src/hardware_interface/ActuatorCoefficient.yaml # path to input yaml file
)

include_directories(include)
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
# ------------------------------------------------ EXECUTABLE ------------------------------------------------ #
set(DEPENDENCIES
    ament_cmake
    rclcpp)
set(EXE_SRC_LIST
  src/node_01.cpp
  src/hardware_interface/CanBus.cpp
  src/hardware_interface/AsyncUsart.cpp
  src/hardware_interface/AsyncSocketCan.cpp
  )
add_executable(${PROJECT_NAME}_exe ${EXE_SRC_LIST})
target_link_libraries(${PROJECT_NAME}_exe PRIVATE actuator_coefficient_lib)
# YAML
ament_target_dependencies(${PROJECT_NAME}_exe PUBLIC ${DEPENDENCIES}) 
set_target_properties(${PROJECT_NAME}_exe PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
# ------------------------------------------------ EXECUTABLE ------------------------------------------------ #
# ------------------------------------------------ UTILITIES ------------------------------------------------ #
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
set (FAKE_PUBLISHER fake_mecanum_joint_publisher)
add_executable(${FAKE_PUBLISHER} src/utilities/fake_mecanum_joint_publisher.cpp) 
ament_target_dependencies(${FAKE_PUBLISHER} PUBLIC ${DEPENDENCIES} realtime_tools sensor_msgs)
# ------------------------------------------------ UTILITIES ------------------------------------------------ #

# ------------------------------------------------- LIBRARY ------------------------------------------------- #
pluginlib_export_plugin_description_file(hardware_interface plugins.xml)

set(PROJECT_SHARE_PATH ${CMAKE_INSTALL_PREFIX})
configure_file(config/config.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h)
set(LIB_SRC_LIST
  src/hardware.cpp
  src/hardware_interface/CanBus.cpp
  src/hardware_interface/AsyncUsart.cpp
  src/hardware_interface/AsyncSocketCan.cpp
)
add_library(${PROJECT_NAME}_lib SHARED ${LIB_SRC_LIST})
target_link_libraries(${PROJECT_NAME}_lib PRIVATE actuator_coefficient_lib yaml-cpp)
ament_target_dependencies(${PROJECT_NAME}_lib PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS}) 
set_target_properties(${PROJECT_NAME}_lib PROPERTIES OUTPUT_NAME ${PROJECT_NAME})
# ------------------------------------------------- LIBRARY ------------------------------------------------- #
install(
  DIRECTORY include/
  DESTINATION include/
)
# https://answers.ros.org/question/363112/how-to-see-compiler-invocation-in-colcon-build/
# message(WARINING "This is config install path: " ${CMAKE_INSTALL_PREFIX})
install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS ${PROJECT_NAME}_exe fake_mecanum_joint_publisher
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)
ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()