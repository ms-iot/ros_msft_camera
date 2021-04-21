cmake_minimum_required(VERSION 3.15.3)
project(ros_msft_camera)

find_package(catkin REQUIRED COMPONENTS
  image_transport roscpp sensor_msgs nodelet camera_info_manager roslint std_msgs roslint)

find_package(Boost REQUIRED COMPONENTS system thread)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CXX_EXTENSIONS FALSE)

add_definitions(/D_SILENCE_ALL_CXX17_DEPRECATION_WARNINGS /DBOOST_BIND_GLOBAL_PLACEHOLDERS)
add_compile_options(/EHsc)

roslint_cpp()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS image_transport roscpp sensor_msgs nodelet camera_info_manager
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  external/Windows-Camera/NetworkMediaStreamer/Common/inc
  )

if(ENABLE_RTSP)
add_definitions(-DENABLE_RTSP)
include(external/CmakeLists.txt)
endif(ENABLE_RTSP)

## Declare a cpp library
add_library(${PROJECT_NAME}_lib STATIC src/winrospublisher.cpp src/wincapture.cpp)

add_library(${PROJECT_NAME}_nodelet SHARED src/msftcameranodelet.cpp)
set_target_properties(${PROJECT_NAME}_nodelet PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE) 

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/msftcameranode.cpp)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_lib)
if(ENABLE_RTSP)
add_dependencies(${PROJECT_NAME}_node RTPMediaStreamer RTSPServer)
endif(ENABLE_RTSP)

add_dependencies(${PROJECT_NAME}_nodelet ${PROJECT_NAME}_lib)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  )

if(ENABLE_RTSP)
set(RTSP_LIBS 
    RTPMediaStreamer 
    RTSPServer)
endif(ENABLE_RTSP)

set (COMMON_LIBS 
     ${PROJECT_NAME}_lib
     mf
     mfplat
     mfuuid
     mfreadwrite
     Mfsensorgroup
     shlwapi
     runtimeobject)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${catkin_LIBRARIES}
  ${COMMON_LIBS}
  )


target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${COMMON_LIBS}
  ${RTSP_LIBS}
  )

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_nodelet ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
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
  target_link_libraries(testMsftCamera ${PROJECT_NAME}_lib ${catkin_LIBRARIES})

  add_rostest_gtest(testMsftCameraNoYaml test/msftCameraNoYaml.test
    test/testMsftCameraNoYaml.cpp)
  target_link_libraries(testMsftCameraNoYaml ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
endif()
