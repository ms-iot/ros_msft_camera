# ros_msft_camera - Windows MF Camera node for ROS
## Introduction
This ROS node uses [Windows Media Foundation API](https://docs.microsoft.com/en-us/windows/win32/medfound/about-the-media-foundation-sdk) to efficiently capture and process video frames from a camera device or an rtsp url.
This node uses [MF SourceReader API](https://docs.microsoft.com/en-us/windows/win32/medfound/source-reader) to read frames from camera device or an RTSP url. The node chooses first available video stream from the camera. Most USB cameras have only one video stream.
The node publishes image frames using image_transport camera publisher on a `image_raw` topic  

This source also contains a feature to stream out the captured video over RTSP/RTP using H.264 video compression for remote teleoperation and telepresence applications.
The streaming feature supports the following:  
- H.264 over RTP (session negotiation via RTSP)
- RTSP digest and basic authentication 
- RTSP over secure(TLS) connection (RTSPS)  
The RTSP/RTP streaming related libraries are a part of a sub-module from the [Windows-Camera repository](https://github.com/microsoft/Windows-Camera/tree/release/NetworkVideoStreamer_1_0)
For more details on streaming [visit this page](https://github.com/microsoft/Windows-Camera/blob/release/NetworkVideoStreamer_1_0/README.md)  

## System Requirement

  * Microsoft Windows 10 64-bit
  * [ROS1 Installation](http://wiki.ros.org/Installation/Windows)
  
## Getting Started

To run this node, a camera will be required to be installed and ready to use on your system.

### Simple camera node
You can begin with the below launch file. It will bring up RViz tool where you can see the image stream from your camera.

```
roslaunch ros_msft_camera ros_msft_camera.launch
```

### Camera node with RTSP/RTP streaming
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
roslaunch ros_msft_camera ros_msft_camera_rtsp.launch
``` 
After the node has been successfully launched with RTSP enabled, you can view the video on any streaming video player that supports RTSP using the url:  
> `rtsp://<ip-address-of-machine-running-the-node>:<rtsp_port>`

If using secure RTSP over TLS, use a video player that supports RTSPS with following url:  
> `rtsps://<ip-address-of-machine-running-the-node>:<rtsp_port>` 

## Published Topics

  * `/image_raw` (/msft_camera_node/image_raw)
    > The image stream from the camera

## Parameters

### Generic parameters  

  * `~image_width` (integer, default: `640`)
    > Desired capture image width.

  * `~image_height` (integer, default: `480`)
    > Desired capture image height.

  * `~frame_rate` (float, default: `30.0`)
    > Desired capture frame rate.

  * `~videoDeviceId` (string, default: ``)
    > Symbolic link to the camera to open. if not set default is the first enumerated camera on the system.
    > Set the Symbolic link to value "Interactive" to configure the node to list the available cameras on system and prompt for user selection at startup

  * `~videoUrl` (string, default: ``)
    > Video URL to be used (instead of an actual camera device) as a source of video frames to be published by the node.
    > At present, file paths and rtsp urls are supported as video source. if both parameters, videoDeviceId and videoUrl, are set then videoDeviceId is used by the node.

  * `~camera_info_url` (string, default: ``)
    > Url to the yaml file with camera distrortion parameters.

For generic parameter usage example see [camnode.launch](test/camnode.launch) 

### RTSP Streaming Parameters  
Only available if the source is built as per the [instructions for streaming](#camera-node-with-rtsprtp-streaming)  
  
  * `~rtsp_port` (integer, default: `8554`)
    > The network port on which the RTSP server should listen for incoming connections.  
    > If this parameter is *not present* in the parameter server, then RTSP server is not started.  
    > If this parameter is *present and empty*, the RTSP server is started with the default port number *8554*

  * `~rtp_bitrate` (integer, default: `1000*image_width`)
    > The bitrate in bps to configure the video encoder used for RTP streaming.
  
  * `~rtsp_AddCredentials` (Dictionary(string: string), default: ``)
    > A dictionary containing username and password credentials to be added to the password vault to be used for RTSP digest authentication. 

  * `~rtsp_RemoveCredentials` (List(string), default: ``)
    > A list containing old usernames to be removed from password vault to be used for RTSP digest authentication. 

  * `~rtsps_cert_subject` (string, default: ``)
    > The certificate subject name to search in the "Local Machine\My" certificate store for RTSP**S** secure streaming over TLS.  
    > Note: To use RTSP**S** (secure RTSP over TLS), a valid server certificate must be imported to the "Local machine\My" certificate store.  
    > If this parameter is *not present*, the stream is not secure and RTSP and RTP communication will happen in the clear.  
    > If this parameter is *present and valid*, RTSP communication will *only* happen securely over TLS. The remote client/player then has the ability to negotiate RTP via the secure TCP channel over RTSP protocol.

For RTSP parameter usage example see [camnodeRTSP.launch](test/camnodeRTSP.launch)

## Remarks

This source also contains a publisher and camera nodelet to enable sharing [IMFSample](https://docs.microsoft.com/en-us/windows/win32/api/mfobjects/nn-mfobjects-imfsample) pointers directly into another nodelet with the same process to enable zero copy and also share GPU surfaces
This is enabled by using the MFSample Publisher which published IMFSample pointer via a custom msg. This path is still experimental and untested.  

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


