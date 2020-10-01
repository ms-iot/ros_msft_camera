cmake_minimum_required(VERSION 3.15.3)
#project(NetworkMediaStreamer)

set (CMAKE_CXX_STANDARD 17)
set (CMAKE_CXX_STANDARD_REQUIRED TRUE)
set (CMAKE_CXX_EXTENSIONS FALSE)

add_library(NetworkMediaStreamerBase STATIC 
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/NetworkMediaStreamerBase/src/NwMediaStreamSinkBase.cpp)

add_library(RTPMediaStreamer SHARED
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTPMediaStreamer/src/RTPMediaSink.cpp
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTPMediaStreamer/src/RTPStreamSink.cpp)

add_library(RTSPServer SHARED
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTSPServer/src/RTSPAuthProvider.cpp
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTSPServer/src/RTSPServer.cpp
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTSPServer/src/RtspSession.cpp
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTSPServer/src/SocketWrapper.cpp)

target_compile_definitions(RTPMediaStreamer PRIVATE RTPMEDIASTREAMER_EXPORTS UNICODE)
target_compile_definitions(RTSPServer PRIVATE RTSPSERVER_EXPORTS UNICODE)

target_include_directories(NetworkMediaStreamerBase PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/Common/inc
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/NetworkMediaStreamerBase/inc)

# The order of the include directories is super-important, because dependancy directories may have a pch.h (meant for the dependancy lib) 
# and the first one in the list is chosen. So make sure the include directory for the correct pch.h comes first 
target_include_directories(RTPMediaStreamer PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/Common/inc
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTPMediaStreamer/inc
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/NetworkMediaStreamerBase/inc
)

target_include_directories(RTSPServer PRIVATE 
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/Common/inc
${CMAKE_CURRENT_SOURCE_DIR}/external/Windows-Camera/NetworkMediaStreamer/RTSPServer/inc)

add_dependencies(RTSPServer RTPMediaStreamer)
add_dependencies(RTPMediaStreamer NetworkMediaStreamerBase)

set(COMMONLIBS 
  mf
  mfplat
  mfuuid
  mfreadwrite
  runtimeobject
  ws2_32) 
  
target_link_libraries(RTSPServer ${COMMONLIBS} Secur32 Crypt32)
target_link_libraries(RTPMediaStreamer ${COMMONLIBS} NetworkMediaStreamerBase)



