// Copyright (C) Microsoft Corporation. All rights reserved.
#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>

#ifdef ENABLE_VIDEOSTREAMING
#include "VideoStreamer.h"
#endif

//#define LOG_LATENCY

using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_win_camera;
constexpr int32_t PUBLISHER_QUEUE_SIZE = 4;
constexpr int32_t DEFAULT_WIDTH = 640;
constexpr int32_t DEFAULT_HEIGHT = 480;
constexpr float DEFAULT_FRAMERATE = 30.0;
auto videoFormat = MFVideoFormat_ARGB32;
#define TEST_RTP_LOOPBACK
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
    privateNode.param("frame_rate", frameRate, DEFAULT_FRAMERATE);
    privateNode.param("pub_queue_size", queueSize, PUBLISHER_QUEUE_SIZE);
    std::condition_variable eventFinish;
    std::mutex mutexFinish;
    std::unique_lock<std::mutex> lockFinish(mutexFinish);
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
            for (auto sym : ros_win_camera::WindowsMFCapture::EnumerateCameraLinks(false))
            {
                std::wcout << i++ << " - " << sym.c_str();
            }
            std::cout << "\nyour choice: ";
            std::cin >> i;
#endif
            // no source path is specified; we default to first enumerated camera
            videoSourcePath = winrt::to_string(ros_win_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(i));
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
    camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath)));
    auto rawPublisher = new WinRosPublisherImageRaw(privateNode, "image_raw", queueSize, frame_id, spCameraInfoManager.get());

    std::string destination, protocol;
    privateNode.param("StreamOutDestination", destination, std::string(""));
    privateNode.param("StreamOutProtocol", protocol, std::string("rtp"));
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

#ifdef ENABLE_VIDEOSTREAMING
    GUID nativeVideoFormat;
    camera->ChangeCaptureConfig(Width, Height, frameRate, GUID_NULL);
    camera->GetCaptureConfig((uint32_t&)Width, (uint32_t&)Height, frameRate, nativeVideoFormat);

    winrt::com_ptr<IVideoStreamer> streamer;
#ifndef TEST_RTP_LOOPBACK
    if (!destination.empty())
#endif
    {
        streamer.attach(CreateFFVideoStreamer());
        streamer->ConfigEncoder(Width, Height, frameRate, nativeVideoFormat, MFVideoFormat_H264, 1000000);
        if (!destination.empty()) streamer->AddDestination(destination);
#ifdef TEST_RTP_LOOPBACK
        //#define RTSP_TEST
#ifdef RTSP_TEST
        system("start cmd /c ffplay.exe -rtsp_flags listen -fflags nobuffer rtsp://127.0.0.1:54455");

        ThreadPool::RunAsync([&](IAsyncAction)
            {
                // start in a separate thread as rtsp push protocol will block on connect to server 
                streamer->AddDestination("127.0.0.1:54455", "rtsp");
            });
#else //RTSP_TEST
        char buf[20000];
        streamer->AddDestination("127.0.0.1:54455", "rtp");
        streamer->GenerateSDP(buf, 20000, "127.0.0.1:54455");
        printf("sdp:\n%s\n", buf);
        FILE* fsdp;
        fopen_s(&fsdp, "test.sdp", "w");
        fprintf(fsdp, "%s", buf);
        fclose(fsdp);

        system("start cmd /c ffplay.exe -protocol_whitelist file,udp,rtp test.sdp");

#endif //RTSP_TEST
#endif //TEST_RTP_LOOPBACK
    }

    auto streamingSampleHandler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
    {
        if (pSample)
        {
#ifdef LOG_LATENCY
            LONGLONG llSampleTime;
            auto tm = MFGetSystemTime();
            pSample->GetSampleTime(&llSampleTime);
            std::cout << "\rSource Delay:" << (tm - llSampleTime) / 10000;
#endif

            if (streamer)
            {
                streamer->WritePacket(pSample);
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
#endif

#ifdef ENABLE_VIDEOSTREAMING
    {
        camera->StartStreaming();
        camera->AddSampleHandler(streamingSampleHandler);

        camera1.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), false));
        camera1->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
        camera1->StartStreaming();
        camera1->AddSampleHandler(rosImagePubHandler);
    }
#else
    camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), true));
    camera->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
    camera->StartStreaming();
    camera->AddSampleHandler(rosImagePubHandler);

#endif
#ifdef TEST_SETCAMERAINFO
    Sleep(10000);
    auto info = spCameraInfoManager->getCameraInfo();
    info.height = 720;
    info.width = 400;
    spCameraInfoManager->setCameraInfo(info);
#endif //#ifdef TEST_SETCAMERAINFO

    ros::spin();

    camera->StopStreaming();
    eventFinish.wait(lockFinish);
    return 0;
}

