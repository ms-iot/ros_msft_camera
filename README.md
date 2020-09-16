# msft_win_camera
## Introduction
This ROS node uses Windows Media Foundation API to efficiently capture and process video frames from a camera device or an rtsp url.
This node uses MF SourceReader API to read frames from camera or rtsp url. The node chooses first available video stream from the camera. Most USB cameras have only one video stream.
The node publishes image frames using image_transport camera publisher on a image_raw topic  

This source also contains camera nodelet to enable sharing IMFSample pointers directly into another nodelet with the same process to enable zero copy and also share GPU surfaces
This is enabled by using the MFSample Publisher which published IMFSample pointer via a custom msg. This path is still experimental and untested.

## How to build

## How to run
useful params:
image_width - desired capture image width, if not set default is 640
image_height - desired capture image height,  if not set default is 480 
frame_rate - desired capture frame rate, if not set, default is 30
videoDeviceId - symbolic link to the camera to open. if not set default is the first enumerated camera on the system 
camera_info_url - url to the yaml file with camera distrortion parameters 


