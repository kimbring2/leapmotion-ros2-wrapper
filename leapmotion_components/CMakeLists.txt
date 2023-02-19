cmake_minimum_required(VERSION 3.5)
project(leapmotion_components)

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

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
)

set(LEAPMOTION_TOOLS_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/leap_tools.hpp
)

#set(LEAPMOTION_TOOLS_SRC
#    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/leap_tools.cpp
#)

set(LEAPMOTION_INC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/leapmotion/include/leapmotion_component.hpp
)

set(LEAPMOTION_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/leapmotion/src/leapmotion_component.cpp
)

set(LIB_LEAP ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/libLeap.so)

add_library(leapmotion_component SHARED
  ${LEAPMOTION_INC}
  ${LEAPMOTION_SRC}
  ${LIB_LEAP}
)

#add_executable(leap_tools 
#  ${LIB_LEAP}
#)

target_link_libraries(leapmotion_component
  ${LIB_LEAP}
  ${PROJECT_SOURCE_DIR}/src/tools/include/Leap.h
)

ament_target_dependencies(leapmotion_component
    rclcpp 
    rclcpp_components
)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include)

# Install components
install(TARGETS leapmotion_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/src/leapmotion/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include/
    DESTINATION include/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(leapmotion_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
