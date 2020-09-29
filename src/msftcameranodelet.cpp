// Copyright (c) Microsoft Corporation.  All rights reserved.

#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>
#include <nodelet/nodelet.h>
using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
namespace ros_msft_camera
{
    const int32_t PUBLISHER_QUEUE_SIZE = 4;

    class MsftCameraNodelet : public nodelet::Nodelet
    {

    public:
        MsftCameraNodelet()
            : m_Width(640),
            m_Height(480),
            m_frameRate(30.0),
            m_QueueSize(PUBLISHER_QUEUE_SIZE)
        {
        }
        ~MsftCameraNodelet()
        {
            m_camera->StopStreaming();
            m_conditionFinish.notify_all();
        }

    private:
        virtual void onInit()
        {
            auto privateNode = getPrivateNodeHandle();
            std::string videoSourcePath = "";
            bool isDevice = true;
            std::string frameId("camera");
            std::string cameraInfoUrl("");
            privateNode.param("frame_rate", m_frameRate, 30.0f);
            privateNode.param("pub_queue_size", m_QueueSize, PUBLISHER_QUEUE_SIZE);

            privateNode.param("image_width", m_Width, 640);
            privateNode.param("image_height", m_Height, 480);

            privateNode.param("frame_id", frameId, std::string("camera"));
            privateNode.getParam("videoDeviceId", videoSourcePath);
            if (videoSourcePath.empty())
            {
                privateNode.getParam("videoUrl", videoSourcePath);
                if (!videoSourcePath.empty())
                {
                    isDevice = false;
                }
                else
                {
                    // no source path is specified; we default to first enumerated camera
                    videoSourcePath = winrt::to_string(ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false).First().Current());
                }
            }

            m_spRawPublisher = std::make_shared<WinRosPublisherImageRaw>(privateNode, "image_raw", m_QueueSize, frameId, m_spCameraInfoManager);
            m_spMFSamplePublisher = std::make_shared<WinRosPublisherMFSample>(privateNode, "MFSample", m_QueueSize, frameId, m_spCameraInfoManager);

            m_spCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(privateNode, "frame_id");
            if (privateNode.getParam("camera_info_url", cameraInfoUrl))
            {
                if (m_spCameraInfoManager->validateURL(cameraInfoUrl))
                {
                    m_spCameraInfoManager->loadCameraInfo(cameraInfoUrl);
                }
            }

            bool resChangeInProgress = false;
            auto handler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
            {
                if (pSample)
                {
                    auto info = m_spCameraInfoManager->getCameraInfo();
                    if (((info.height != m_Height) || (info.width != m_Width)) && info.height && info.width && (!resChangeInProgress))
                    {
                        resChangeInProgress = true;
                        ThreadPool::RunAsync([this, &resChangeInProgress](IAsyncAction)
                            {
                                auto info = m_spCameraInfoManager->getCameraInfo();
                                if (m_camera->ChangeCaptureConfig(info.width, info.height, m_frameRate, MFVideoFormat_ARGB32, true))
                                {
                                    m_Height = info.height;
                                    m_Width = info.width;
                                    resChangeInProgress = false;
                                }
                                else
                                {
                                    ROS_WARN("Setting resolution  failed. Use rescale_camera_info param for rescaling");
                                }
                            });
                    }
                    else
                    {
                        m_spRawPublisher->OnSample(pSample, m_Width, m_Height);
                        m_spMFSamplePublisher->OnSample(pSample, (UINT32)m_Width, (UINT32)m_Height);
                    }
                }
                else
                {
                    if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
                    {
                        ROS_INFO("\nEOS");
                    }
                    m_conditionFinish.notify_all();
                }
            };

            std::mutex mutexFinish;

            std::unique_lock<std::mutex> ul(mutexFinish);
            m_conditionFinish.wait(ul);
            m_camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), true));
            
            m_camera->StartStreaming();
            m_camera->ChangeCaptureConfig(m_Width, m_Height, m_frameRate, MFVideoFormat_ARGB32);
            m_camera->AddSampleHandler(handler);
            

        }
        std::condition_variable m_conditionFinish;
        winrt::com_ptr< ros_msft_camera::WindowsMFCapture> m_camera;
        int32_t m_Width, m_Height;
        int32_t m_QueueSize;
        float m_frameRate;
        std::shared_ptr<camera_info_manager::CameraInfoManager> m_spCameraInfoManager;
        std::shared_ptr<WinRosPublisherImageRaw> m_spRawPublisher;
        std::shared_ptr<WinRosPublisherMFSample> m_spMFSamplePublisher;
    };
}