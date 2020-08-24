# ROS 2 Windows Media Foundation Camera Driver

This ROS node uses Windows Media Foundation's frame server to efficiently process camera frames.
This node uses MF SourceReader API to read frames from camera. The node chooses first available video stream from the camera. Most USB cameras have only one video stream.
Node publishes using image_transport Camera publisher on `image_raw` topic.

## System Requirement

  * Microsoft Windows 10 64-bit
  * ROS2 Installation (`Foxy` is recommended.)

## Getting Started

To run this driver, a camera will be required to be installed and ready to use on your system.

You can begin with the below launch file. It will bring up RViz tool where you can see the image stream from your camera.

```Batchfile
ros2 launch win_camera win_camera.launch.py
```

In addtions, this driver is registered as a ROS 2 component and it can be running inside a [component container](https://index.ros.org/doc/ros2/Tutorials/Composition/). The below is a demonstration for the usage.

```Batchfile
ros2 launch win_camera win_camera_components.launch.py
```

## Published Topics

  * `/image_raw` (sensor_msgs/msg/Image)
    > The image stream from the camera.

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
