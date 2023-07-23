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
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(DEPENDENCIES
    ament_cmake
    rclcpp)
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

include_directories(include)
set(SRC_LIST
  src/node_01.cpp
  src/hardware_interface/CanBus.cpp
  src/hardware_interface/AsyncUsart.cpp
  src/hardware_interface/AsyncSocketCan.cpp)
add_executable(${PROJECT_NAME} ${SRC_LIST})
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES}) 

install(
  DIRECTORY include/
  DESTINATION include/
)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(${DEPENDENCIES})
ament_package()