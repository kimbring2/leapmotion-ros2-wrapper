cmake_minimum_required(VERSION 3.5)
project(leapmotion_wrapper)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(leapmotion_components REQUIRED)
find_package(example_interfaces REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

file(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${all_files})

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(LIB_LEAP ${CMAKE_CURRENT_SOURCE_DIR}/include/libLeap.so)

add_executable(leapmotion_wrapper src/leapmotion_wrapper.cpp)

ament_target_dependencies(leapmotion_wrapper
rclcpp 
rclcpp_components
leapmotion_components
example_interfaces)

install(TARGETS leapmotion_wrapper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(leapmotion_wrapper
  ${LIB_LEAP}
  ${PROJECT_SOURCE_DIR}/include/Leap.h
)

#install(TARGETS
#  leapmotion_wrapper
#  DESTINATION lib/${PROJECT_NAME})


# Install PARAMS files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)


# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)


# Install LAUNCH files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()