# ros_win_camera
This ROS node uses Windows Media Foundation's frame server to efficiently process camera frames.
This node uses MF SourceReader API to read frames from camera. The node chooses first available video stream from the camera. Most USB cameras have only one video stream.
Node publishes using image_transport Camera publisher on image_raw topic  

useful params:
image_width - desired capture image width, if not set default is 640
image_height - desired capture image height,  if not set default is 480 
frame_rate - desired capture frame rate, if not set, default is 30
videoDeviceId - symbolic link to the camera to open. if not set default is the first enumerated camera on the system 
camera_info_url - url to the yaml file with camera distrortion parameters 

This source also contains camera nodelet to enable sharing IMFSample pointers directly into another nodelet with the same process to enable zero copy and also share GPU surfaces
This is enabled by using the MFSample Publisher which published IMFSample pointer via a custom msg. This path is still experimental and untested.

