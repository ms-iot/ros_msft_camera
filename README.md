# ros_msft_camera - Windows MF Camera node for ROS
## Introduction
This ROS node uses Windows Media Foundation API to efficiently capture and process video frames from a camera device or an rtsp url.
This node uses MF SourceReader API to read frames from camera or rtsp url. The node chooses first available video stream from the camera. Most USB cameras have only one video stream.
The node publishes image frames using image_transport camera publisher on a `image_raw` topic  

This source also contains camera nodelet to enable sharing IMFSample pointers directly into another nodelet with the same process to enable zero copy and also share GPU surfaces
This is enabled by using the MFSample Publisher which published IMFSample pointer via a custom msg. This path is still experimental and untested.

This source also contains feature to stream the video over RTSP/RTP using H.264 video compression.
The RTSP/RTP streaming code is a sub-module from the [Windows-Camera repository](https://github.com/microsoft/Windows-Camera/tree/release/NetworkVideoStreamer_1_0)
## System Requirement

  * Microsoft Windows 10 64-bit
  * [ROS1 Installation](http://wiki.ros.org/Installation/Windows) (`Melodic` is recommended.)
  
## Getting Started

To run this node, a camera will be required to be installed and ready to use on your system.

You can begin with the below launch file. It will bring up RViz tool where you can see the image stream from your camera.

```
roslaunch msft_camera test/camnode.launch
```

To enable RTSP streaming you will need to create a recursive clone of this repository using the following command:  
```
git clone https://github.com/ms-iot/ros_msft_camera --recursive
```

Then at the top level folder (i.e. parallel to the src folder) use the command  

```
catkin_make -DENABLE_RTSP=1
```
This will build the node with RTP/RTSP streaming capability. 
To launch the node with streaming enabled, use the following command:

```
roslaunch msft_camera_rtsp test/camnodeRTSP.launch
``` 

## Published Topics

  * `/image_raw` (/msft_camera_node/image_raw)
    > The image stream from the camera

## Parameters

  * `~image_width` (integer, default: `640`)
    > Desired capture image width.

  * `~image_height` (integer, default: `480`)
    > Desired capture image height.

  * `~frame_rate` (float, default: `30.0`)
    > Desired capture frame rate.

  * `~videoDeviceId` (string, default: ``)
    > Symbolic link to the camera to open. if not set default is the first enumerated camera on the system.

  * `~camera_info_url` (string, default: ``)
    > Url to the yaml file with camera distrortion parameters.

## Remarks

This source also contains camera components to enable sharing IMFSample pointers directly into another component container with the same process to enable zero copy and also share GPU surfaces.
This is enabled by using the MFSample Publisher which published IMFSample pointer via a custom msg. This path is experimental.

# Contributing

This project welcomes contributions and suggestions.  Most contributions require you to agree to a
Contributor License Agreement (CLA) declaring that you have the right to, and actually do, grant us
the rights to use your contribution. For details, visit https://cla.microsoft.com.

When you submit a pull request, a CLA-bot will automatically determine whether you need to provide
a CLA and decorate the PR appropriately (e.g., label, comment). Simply follow the instructions
provided by the bot. You will only need to do this once across all repos using our CLA.

This project has adopted the [Microsoft Open Source Code of Conduct](https://opensource.microsoft.com/codeofconduct/).
For more information see the [Code of Conduct FAQ](https://opensource.microsoft.com/codeofconduct/faq/) or
contact [opencode@microsoft.com](mailto:opencode@microsoft.com) with any additional questions or comments.


