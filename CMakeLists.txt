cmake_minimum_required(VERSION 3.15.3)
project(msft_camera)

find_package(catkin REQUIRED COMPONENTS
  image_transport roscpp sensor_msgs nodelet camera_info_manager roslint std_msgs message_generation roslint)

find_package(Boost REQUIRED COMPONENTS system thread)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CXX_EXTENSIONS FALSE)

roslint_cpp()
add_message_files(
   FILES
   MFSample.msg
 )
generate_messages(
   DEPENDENCIES
   std_msgs
 )
  
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES win_camera
  CATKIN_DEPENDS image_transport roscpp sensor_msgs nodelet camera_info_manager message_runtime
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  external/Windows-Camera/Samples/NetworkMediaStreamer/Common/inc
  )

#if(ENABLE_RTSP)
include(external/CmakeLists.txt)
#endif(ENABLE_RTSP)
## Declare a cpp library
add_library(win_camera STATIC src/winrospublisher.cpp src/wincapture.cpp)

add_dependencies(win_camera msft_camera_generate_messages_cpp)

add_library(msft_camera_nodelet SHARED src/msftcameranodelet.cpp)
set_target_properties(msft_camera_nodelet PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE) 

## Declare a cpp executable
add_executable(msft_camera_node src/msftcameranode.cpp)
add_dependencies(msft_camera_node win_camera RTPMediaStreamer RTSPServer)
add_dependencies(msft_camera_nodelet win_camera)

## Specify libraries to link a library or executable target against
target_link_libraries(win_camera
  ${catkin_LIBRARIES}
  )

target_link_libraries(msft_camera_nodelet
  ${catkin_LIBRARIES}
  win_camera
  mf
  mfplat
  mfuuid
  mfreadwrite
  Mfsensorgroup
  shlwapi
  runtimeobject
  )

target_link_libraries(msft_camera_node
  ${catkin_LIBRARIES}
  win_camera
  mf
  mfplat
  mfuuid
  mfreadwrite
  Mfsensorgroup
  shlwapi
  runtimeobject
  RTPMediaStreamer 
  RTSPServer
  )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS win_camera msft_camera_nodelet msft_camera_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

# add xml file
install(FILES msft_camera_nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )


#############
## Testing ##
#############

roslint_cpp(
  src/msftcameranode.cpp src/msftcameranodelet.cpp src/wincapture.cpp src/winrospublisher.cpp
  )

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(testMsftCamera test/msftCamera.test
    test/testMsftCamera.cpp)
  target_link_libraries(testMsftCamera win_camera ${catkin_LIBRARIES})

  add_rostest_gtest(testMsftCameraNoYaml test/msftCameraNoYaml.test
    test/testMsftCameraNoYaml.cpp)
  target_link_libraries(testMsftCameraNoYaml win_camera ${catkin_LIBRARIES})
endif()
