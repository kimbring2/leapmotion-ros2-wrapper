cmake_minimum_required(VERSION 3.5)
project(leapmotion_interfaces)

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

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

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


# Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
file(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_${PROJECT_NAME}_files SOURCES ${extra_files})

set(MSG_FILES
    "msg/Object.msg"
    "msg/ObjectsStamped.msg"
    "msg/Keypoint2Di.msg"
    "msg/Keypoint2Df.msg"
    "msg/Keypoint3D.msg"
    "msg/BoundingBox2Di.msg"
    "msg/BoundingBox2Df.msg"
    "msg/BoundingBox3D.msg"
    "msg/Skeleton2D.msg"
    "msg/Skeleton3D.msg"
    "msg/DepthInfoStamped.msg"
    "msg/PlaneStamped.msg"
)

set(SRV_FILES
    "srv/SetPose.srv"
    "srv/StartSvoRec.srv"
    "srv/SetROI.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
    ${MSG_FILES}
    ${SRV_FILES}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs shape_msgs
)

# Install URDF files
install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
