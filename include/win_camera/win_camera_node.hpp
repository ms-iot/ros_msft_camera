// Copyright (C) Microsoft Corporation. All rights reserved.

#ifndef WIN_CAMERA__WIN_CAMERA_NODE_HPP_
#define WIN_CAMERA__WIN_CAMERA_NODE_HPP_

#include <mfidl.h>
#include <winrt/base.h>
#include <win_camera/wincapture.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

#include <rclcpp/rclcpp.hpp>

#include <string>

#include <win_camera/visibility_control.hpp>

namespace win_camera
{

class WIN_CAMERA_PUBLIC_TYPE CameraDriver final
  : public rclcpp::Node
{
public:
    /// \brief Default constructor
    explicit CameraDriver(const rclcpp::NodeOptions & options);
    ~CameraDriver();

private:
    void StartCameraStreaming();
    void StopCameraStreaming();
    void OnSample(IMFSample *pSample, UINT32 u32Width, UINT32 u32Height);
    void RescaleCameraInfo(sensor_msgs::msg::CameraInfo &cameraInfo, int width, int height);

    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    winrt::com_ptr<ros_win_camera::WindowsMFCapture> camera_;
    std::string frame_id_;
    int32_t image_width_;
    int32_t image_height_;
    float frame_rate_;
    int32_t pub_queue_size_;
    std::string video_source_path_;
    bool isDevice;
    std::string camera_info_url_;
    std::condition_variable event_finish_;
    bool m_bRescaleCameraInfo;
    image_transport::CameraPublisher m_cameraPublisher;
};

} // namespace win_camera

#endif // WIN_CAMERA__WIN_CAMERA_NODE_HPP_