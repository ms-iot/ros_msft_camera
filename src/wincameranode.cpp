// Copyright (C) Microsoft Corporation. All rights reserved.
#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>
#include "ffrtp.h"
extern uint32_t g_dropCount;
using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_win_camera;
const int32_t PUBLISHER_QUEUE_SIZE = 4;
auto videoFormat = MFVideoFormat_RGB24;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "win_camera");
    ros::NodeHandle privateNode("~");
    std::string videoSourcePath = "";
    bool isDevice = true;
    float frameRate = 30;
    int32_t queueSize = PUBLISHER_QUEUE_SIZE;
    winrt::com_ptr< ros_win_camera::WindowsMFCapture> camera, camera1;
    std::string frame_id("camera");
    privateNode.param("frame_rate", frameRate, 30.0f);
    privateNode.param("pub_queue_size", queueSize, PUBLISHER_QUEUE_SIZE);
    std::mutex waitForFinish;
    int32_t Width(640), Height(480);
    std::string cameraInfoUrl("");
    privateNode.param("image_width", Width, 640);
    privateNode.param("image_height", Height, 480);

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
            // no source path is specified; we default to first enumerated camera
            videoSourcePath = winrt::to_string(ros_win_camera::WindowsMFCapture::EnumerateCameraLinks(false).First().Current());
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
            int slashCnt=0;
            while (cameraInfoUrl[++pos] == '/') slashCnt++;
            if (cameraInfoUrl[pos] == '\\' && cameraInfoUrl[pos+1] == '\\')
            {
                // unc path that needs conversion
                auto path = cameraInfoUrl.substr(pos+2);
                cameraInfoUrl = std::string("file:////") + path;
            }
            else if (cameraInfoUrl[pos + 1] == ':')
            {
                //full drive path
                auto path = cameraInfoUrl.substr(pos);
                path[1] = '$'; // replace ':' with '$' to convert to unc path
                cameraInfoUrl = std::string("file:////127.0.0.1\\")+path;
            }
            else if(slashCnt < 4) // if slashCnt >= 4 it is probably is an acceptable unc path
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
    camera.attach(new ros_win_camera::WindowsMFCapture(isDevice, winrt::to_hstring(videoSourcePath)));
    WinRosPublisherImageRaw rawPublisher(privateNode, "image_raw", queueSize, frame_id, spCameraInfoManager.get());
    winrt::com_ptr<IMFSinkWriter> spSinkWriter;
    //spSinkWriter.attach(ConfigRTP(Width, Height, frameRate, MFVideoFormat_H264, videoFormat, 1000000, "rtp://127.0.0.1:49995"));
    RTPStreamer streamer(Width, Height, frameRate, MFVideoFormat_H264, videoFormat, 1000000, "127.0.0.1:49995");
    bool resChangeInProgress = false;
    auto handler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
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
                rawPublisher.OnSample(pSample, (UINT32)Width, (UINT32)Height);
                LONGLONG llSampleTime;
                auto tm = MFGetSystemTime();
                pSample->GetSampleTime(&llSampleTime);
                /*if (g_dropCount)
                {
                    g_dropCount--;
                }
                else*/
                {
                    streamer.WritePacket(pSample);
                }
            }
        }
        else
        {
            if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
            {
                ROS_INFO("\nEOS");
            }
            waitForFinish.unlock();
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
            waitForFinish.unlock();
        }
    };

    waitForFinish.lock();

    camera->StartStreaming();
    if (!camera->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_MJPG))
    {
        camera->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
        camera->StartStreaming();
        camera->AddSampleHandler(handler);
    }
    else
    {
        camera->StartStreaming();
        camera->AddSampleHandler(compressedSampleHandler);

        camera1.attach(new ros_win_camera::WindowsMFCapture(isDevice, winrt::to_hstring(videoSourcePath), false));
        camera1->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
        camera1->StartStreaming();
        camera1->AddSampleHandler(handler);
    }
#ifdef TEST_SETCAMERAINFO
    Sleep(10000);
    auto info = spCameraInfoManager->getCameraInfo();
    info.height = 720;
    info.width = 400;
    spCameraInfoManager->setCameraInfo(info);
#endif //#ifdef TEST_SETCAMERAINFO
    
    ros::spin();

    camera->StopStreaming();
    waitForFinish.lock();
    return 0;
}

