// Copyright (C) Microsoft Corporation. All rights reserved.
#include <wincapture.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>

using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_win_camera;
constexpr int32_t PUBLISHER_QUEUE_SIZE = 4;
constexpr int32_t DEFAULT_WIDTH = 640;
constexpr int32_t DEFAULT_HEIGHT = 480;
constexpr float DEFAULT_FRAMERATE = 30.0;
auto videoFormat = MFVideoFormat_ARGB32;

class CameraDriver : public rclcpp::Node
{
  public:
    CameraDriver()
    : Node("win_camera_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)),
    isDevice(true),
    frame_id_("camera"),
    frame_rate_(DEFAULT_FRAMERATE),
    pub_queue_size_(PUBLISHER_QUEUE_SIZE),
    image_width_(DEFAULT_WIDTH),
    image_height_(DEFAULT_HEIGHT),
    video_source_path_("")
    {
        this->get_parameter("frame_id", frame_id_);
        this->get_parameter("frame_rate", frame_rate_);
        this->get_parameter("pub_queue_size", pub_queue_size_);
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);
        this->get_parameter("videoDeviceId", video_source_path_);
        this->get_parameter("camera_info_url", camera_info_url_);
        this->get_parameter("rescale_camera_info", m_bRescaleCameraInfo);
        if (video_source_path_.empty())
        {
            this->get_parameter("videoUrl", video_source_path_);
            if (!video_source_path_.empty())
            {
                isDevice = false;
            }
            else
            {
                int i = 0;
                // no source path is specified; we default to first enumerated camera
                video_source_path_ = winrt::to_string(ros_win_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(i));
            }
        }
        camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, frame_id_, "");
        if (!camera_info_url_.empty())
        {
            if (camera_info_manager_->validateURL(camera_info_url_))
            {
                RCLCPP_INFO(this->get_logger(), "validated Camera info url: %s", camera_info_url_.c_str());
                camera_info_manager_->loadCameraInfo(camera_info_url_);
            }
        }

        m_cameraPublisher = image_transport::create_camera_publisher(this, "image_raw");
    }

    void StartCameraStreaming()
    {
        bool resChangeInProgress = false;
        auto rosImagePubHandler = [this, &resChangeInProgress](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
        {
            if (pSample)
            {
                auto info = this->camera_info_manager_->getCameraInfo();
                if (((info.height != this->image_height_) || (info.width != this->image_width_)) && info.height && info.width && (!resChangeInProgress))
                {
                    resChangeInProgress = true;
                    ThreadPool::RunAsync([this, &resChangeInProgress](IAsyncAction)
                        {
                            auto info = this->camera_info_manager_->getCameraInfo();
                            if (this->camera_->ChangeCaptureConfig(info.width, info.height, this->frame_rate_, videoFormat, true))
                            {
                                this->image_height_ = info.height;
                                this->image_width_ = info.width;
                                resChangeInProgress = false;
                            }
                            else
                            {
                                RCLCPP_WARN(this->get_logger(), "Setting resolution  failed. Use rescale_camera_info param for rescaling");
                            }
                        });
                }
                else
                {
                    OnSample(pSample, (UINT32)this->image_width_, (UINT32)this->image_height_);
                }
            }
            else
            {
                if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
                {
                    RCLCPP_INFO(this->get_logger(), "EOS");
                }
                this->event_finish_.notify_all();
            }
        };

        camera_.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(video_source_path_), true));
        camera_->ChangeCaptureConfig(image_width_, image_height_, frame_rate_, videoFormat, true);
        camera_->StartStreaming();
        camera_->AddSampleHandler(rosImagePubHandler);
    }

    void StopCameraStreaming()
    {
        std::mutex mutexFinish;
        std::unique_lock<std::mutex> lockFinish(mutexFinish);
        ThreadPool::RunAsync([this](IAsyncAction)
            {
                this->camera_->StopStreaming();
            });
        event_finish_.wait(lockFinish);
    }

  private:
    void OnSample(IMFSample *pSample, UINT32 u32Width, UINT32 u32Height)
    {
        try 
        {
            if (pSample)
            {
                winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
                winrt::com_ptr<IMF2DBuffer2> spMediaBuf2d;
                uint32_t littleEndian = 1;
                BYTE* pix;
                LONG Stride;

                auto cameraInfo = camera_info_manager_->getCameraInfo();
                if (cameraInfo.height == 0 && cameraInfo.width == 0)
                {
                    cameraInfo.height = u32Height;
                    cameraInfo.width = u32Width;
                }
                else if (cameraInfo.height != u32Height || cameraInfo.width != u32Width)
                {
                    if (m_bRescaleCameraInfo)
                    {
                        int old_width = cameraInfo.width;
                        int old_height = cameraInfo.height;
                        RescaleCameraInfo(cameraInfo, u32Width, u32Height);
                        RCLCPP_INFO_ONCE(this->get_logger(), "Camera calibration automatically rescaled from %dx%d to %dx%d",
                            old_width, old_height, u32Width, u32Height);
                    }
                    else
                    {
                        RCLCPP_WARN_ONCE(this->get_logger(), "Calibration resolution %dx%d does not match camera resolution %dx%d. "
                            "Use rescale_camera_info param for rescaling",
                            cameraInfo.width, cameraInfo.height, u32Width, u32Height);
                    }
                }
                cameraInfo.header.stamp = rclcpp::Time();
                cameraInfo.header.frame_id = frame_id_;

                winrt::check_hresult(pSample->GetBufferByIndex(0, spMediaBuf.put()));
                spMediaBuf2d = spMediaBuf.as<IMF2DBuffer2>();

                auto ros_image = std::make_shared<sensor_msgs::msg::Image>();

                ros_image->header = cameraInfo.header;
                ros_image->height = u32Height;
                ros_image->width = u32Width;
                ros_image->encoding = sensor_msgs::image_encodings::BGRA8;
                ros_image->is_bigendian = !*((uint8_t*)&littleEndian);

                winrt::check_hresult(spMediaBuf2d->Lock2D(&pix, &Stride));
                if (Stride < 0)
                {
                    ros_image->step = -Stride;
                    size_t size = ros_image->step * u32Height;
                    ros_image->data.resize(size);
                    for (int i = 0; i < u32Height; i++)
                    {
                        memcpy(ros_image->data.data(), pix + Stride * i, -Stride);
                    }
                }
                else
                {
                    ros_image->step = Stride;
                    size_t size = Stride * u32Height;
                    ros_image->data.resize(size);
                    memcpy(ros_image->data.data(), pix, size);

                }
                winrt::check_hresult(spMediaBuf2d->Unlock2D());
                m_cameraPublisher.publish(*ros_image, cameraInfo);
            }
        }
        catch (winrt::hresult_error const& ex)
        {
            std::cout << ex.code() << ex.message().c_str();
        }
    }

    void RescaleCameraInfo(sensor_msgs::msg::CameraInfo &cameraInfo, int width, int height)
    {
        double widthCoeff = static_cast<double>(width) / cameraInfo.width;
        double heightCoeff = static_cast<double>(height) / cameraInfo.height;
        cameraInfo.width = width;
        cameraInfo.height = height;

        // ref: https://github.com/ros2/common_interfaces/blob/master/sensor_msgs/msg/CameraInfo.msg
        cameraInfo.k[0] *= widthCoeff;
        cameraInfo.k[2] *= widthCoeff;
        cameraInfo.k[4] *= heightCoeff;
        cameraInfo.k[5] *= heightCoeff;

        cameraInfo.p[0] *= widthCoeff;
        cameraInfo.p[2] *= widthCoeff;
        cameraInfo.p[5] *= heightCoeff;
        cameraInfo.p[6] *= heightCoeff;
    }

    std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
    winrt::com_ptr< ros_win_camera::WindowsMFCapture> camera_;
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto publisher = std::make_shared<CameraDriver>();

    publisher->StartCameraStreaming();

    rclcpp::spin(publisher);
    rclcpp::shutdown();

    publisher->StopCameraStreaming();

    return 0;
}
