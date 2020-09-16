// Copyright (C) Microsoft Corporation. All rights reserved.
#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>

using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_msft_camera;
constexpr int32_t PUBLISHER_QUEUE_SIZE = 4;
constexpr int32_t DEFAULT_WIDTH = 640;
constexpr int32_t DEFAULT_HEIGHT = 480;
constexpr float DEFAULT_FRAMERATE = 30.0;
auto videoFormat = MFVideoFormat_ARGB32;
int main(int argc, char** argv)
{
    try
    {
        ros::init(argc, argv, "msft_camera");
        ros::NodeHandle privateNode("~");
        std::string videoSourcePath = "";
        bool isDevice = true;
        float frameRate = 30;
        int32_t queueSize = PUBLISHER_QUEUE_SIZE;
        winrt::com_ptr< ros_msft_camera::WindowsMFCapture> camera, camera1;
        std::string frame_id("camera");
        privateNode.param("frame_rate", frameRate, DEFAULT_FRAMERATE);
        privateNode.param("pub_queue_size", queueSize, PUBLISHER_QUEUE_SIZE);
        std::condition_variable eventFinish;
        int32_t Width(DEFAULT_WIDTH), Height(DEFAULT_HEIGHT);
        std::string cameraInfoUrl("");
        privateNode.param("image_width", Width, DEFAULT_WIDTH);
        privateNode.param("image_height", Height, DEFAULT_HEIGHT);

        privateNode.param("frame_id", frame_id, std::string("camera"));

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
                int i = 0;
#ifdef INTERACTIVE
                std::map< std::string, ros::console::levels::Level> logger;
                ros::console::get_loggers(logger);
                if (logger[ROSCONSOLE_DEFAULT_NAME] == ros::console::levels::Level::Info)
                {
                    for (auto sym : ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false))
                    {
                        ROS_INFO("%d. %s", i++, sym.c_str());
                    }
                    ROS_INFO("\nYour choice: ");
                    std::cin >> i;
                }
                else
                {
                    ROS_ERROR("\nINTERATIVE mode for camera selection needs logger levels set atleast to Level::Info");
                }
#endif
                // no source path is specified; we default to first enumerated camera
                videoSourcePath = winrt::to_string(ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(i));
            }
        }
        std::shared_ptr<camera_info_manager::CameraInfoManager> spCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(privateNode, frame_id, "");
        if (privateNode.getParam("camera_info_url", cameraInfoUrl))
        {
            auto pos = cameraInfoUrl.find("file:");
            if (pos != std::string::npos)
            {
                // this block of code is required because there is a bug in camerainfo manager url based api 
                // when handling file:// prefix url with local paths like "e:\folder"
                // We convert such paths to unc path.

                pos += 5; // length of "file:"
                int slashCnt = 0;
                while (cameraInfoUrl[++pos] == '/') slashCnt++;
                if (cameraInfoUrl[pos] == '\\' && cameraInfoUrl[pos + 1] == '\\')
                {
                    // unc path that needs conversion
                    auto path = cameraInfoUrl.substr(pos + 2);
                    cameraInfoUrl = std::string("file:////") + path;
                }
                else if (cameraInfoUrl[pos + 1] == ':')
                {
                    //full drive path
                    auto path = cameraInfoUrl.substr(pos);
                    path[1] = '$'; // replace ':' with '$' to convert to unc path
                    cameraInfoUrl = std::string("file:////127.0.0.1\\") + path;
                }
                else if (slashCnt < 4) // if slashCnt >= 4 it is probably is an acceptable unc path
                {
                    ROS_ERROR("camera info url for file must be a fully qualified drive path or unc path");
                }
            }

            if (spCameraInfoManager->validateURL(cameraInfoUrl))
            {
                ROS_INFO("validated Camera info url: %s", cameraInfoUrl.c_str());
                spCameraInfoManager->loadCameraInfo(cameraInfoUrl);
            }
        }
        //camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath)));
        auto rawPublisher = new WinRosPublisherImageRaw(privateNode, "image_raw", queueSize, frame_id, spCameraInfoManager.get());

        bool resChangeInProgress = false;
        auto rosImagePubHandler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
        {
            if (pSample)
            {
                auto info = spCameraInfoManager->getCameraInfo();
                if (((info.height != Height) || (info.width != Width)) && info.height && info.width && (!resChangeInProgress))
                {
                    resChangeInProgress = true;
                    ThreadPool::RunAsync([camera, &Width, &Height, frameRate, spCameraInfoManager, &resChangeInProgress](IAsyncAction)
                        {
                            auto info = spCameraInfoManager->getCameraInfo();
                            if (camera->ChangeCaptureConfig(info.width, info.height, frameRate, videoFormat, true))
                            {
                                Height = info.height;
                                Width = info.width;
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
                    rawPublisher->OnSample(pSample, (UINT32)Width, (UINT32)Height);
                }
            }
            else
            {
                if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
                {
                    ROS_INFO("\nEOS");
                }
                eventFinish.notify_all();
            }
        };

        auto compressedSampleHandler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
        {
            if (pSample)
            {
                ROS_INFO("Received SAmple compressed\n");

            }
            else
            {
                if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
                {
                    ROS_INFO("\nEOS");
                }
                eventFinish.notify_all();
            }
        };

        try
        {
            camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), true));
        }
        catch (hresult_error const& ex)
        {
            ROS_WARN("WindowsMFCapture creation failed. Retrying with shared mode...");
            // if creation failed, re-try with sharing mode
            camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), false));
            ROS_WARN("WindowsMFCapture is in shared mode. Config paramters may not be applied ");
        }
        //camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), true));
        camera->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
        camera->StartStreaming();
        camera->AddSampleHandler(rosImagePubHandler);

#ifdef TEST_SETCAMERAINFO
        Sleep(10000);
        auto info = spCameraInfoManager->getCameraInfo();
        info.height = 720;
        info.width = 400;
        spCameraInfoManager->setCameraInfo(info);
#endif //#ifdef TEST_SETCAMERAINFO

        ros::spin();

        std::mutex mutexFinish;
        std::unique_lock<std::mutex> lockFinish(mutexFinish);
        ThreadPool::RunAsync([camera](IAsyncAction)
            {
                camera->StopStreaming();
            });
        eventFinish.wait(lockFinish);
        return 0;
    }
    catch (hresult_error const& ex)
    {
        ROS_ERROR(winrt::to_string(ex.message() + L":" + winrt::to_hstring(ex.code())).c_str());
    }
}
