cmake_minimum_required(VERSION 3.15.3)
project(win_camera)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_CXX_STANDARD_REQUIRED)
  set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
endif()

if(NOT CMAKE_CXX_EXTENSIONS)
  set(CMAKE_CXX_EXTENSIONS FALSE)
endif()

if(MSVC)
  add_compile_definitions(
    NOMINMAX
  )
endif()

find_package(ament_cmake REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(compressed_image_transport REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_msgs
  "msg/MFSample.msg"
  DEPENDENCIES std_msgs
)

include_directories(
  include
)

## Declare a cpp library
add_library(win_camera STATIC src/wincapture.cpp)

ament_target_dependencies(win_camera
  compressed_image_transport
  image_transport
  camera_info_manager
  sensor_msgs
  rclcpp
)

## Declare a cpp executable
add_executable(win_camera_node src/wincameranode.cpp)

target_link_libraries(win_camera_node
  win_camera
  mf
  mfplat
  mfuuid
  mfreadwrite
  Mfsensorgroup
  shlwapi
  runtimeobject
)

ament_target_dependencies(win_camera_node
  compressed_image_transport
  image_transport
  camera_info_manager
  sensor_msgs
  rclcpp
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS win_camera win_camera_node
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

install(DIRECTORY include
  DESTINATION include
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  camera_info
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
