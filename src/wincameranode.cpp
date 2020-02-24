#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>
using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_win_camera;
const int32_t PUBLISHER_BUFFER_SIZE = 4;
int main(int argc, char** argv)
{
    ros::init(argc, argv, "win_camera");
    ros::NodeHandle privateNode("~");
    std::string videoSourcePath = "";
    bool isDevice = true;
    float frameRate = 30;
    winrt::com_ptr< ros_win_camera::WindowsMFCapture> camera, camera1;
    std::string frame_id("camera");
    privateNode.getParam("frame_rate", frameRate);

    winrt::slim_mutex waitForFinish;
    int32_t Width(640), Height(480);

    if (!((privateNode.getParam("image_width", Width))
        && (privateNode.getParam("image_height", Height)))
        )
    {
        Width = 640;
        Height = 480;
    }

    privateNode.getParam("frame_id", frame_id);
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
    std::shared_ptr<camera_info_manager::CameraInfoManager> spCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(privateNode, "frame_id");
    WinRosPublisherImageRaw rawPublisher(privateNode, "image_raw", PUBLISHER_BUFFER_SIZE, frame_id, spCameraInfoManager.get());
    WinRosPublisherMFSample mfSamplePublisher(privateNode, "MFSample", PUBLISHER_BUFFER_SIZE, frame_id, spCameraInfoManager.get());
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
                        if (camera->ChangeCaptureConfig(info.width, info.height, frameRate, MFVideoFormat_ARGB32, true))
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
                //castd::cout << "Received SAmple\n";
                rawPublisher.OnSample(pSample, (UINT32)Width, (UINT32)Height);
                mfSamplePublisher.OnSample(pSample, (UINT32)Width, (UINT32)Height);
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

    auto handler1 = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
    {
        if (pSample)
        {
            std::cout << "Received SAmple compressed\n";

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

    camera.attach(new ros_win_camera::WindowsMFCapture(isDevice, winrt::to_hstring(videoSourcePath)));
    camera->StartStreaming();
    //camera->StopStreaming();
    if (!camera->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_MJPG))
    {
        camera->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_ARGB32, true);
        camera->StartStreaming();
        camera->AddSampleHandler(handler);
    }
    else
    {
        camera->StartStreaming();
        camera->AddSampleHandler(handler1);

        camera1.attach(new ros_win_camera::WindowsMFCapture(isDevice, winrt::to_hstring(videoSourcePath), false));
        camera1->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_ARGB32, true);
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
    std::cout << "\nPress enter key to stop";
    std::cin.get();
    camera->StopStreaming();
    waitForFinish.lock();
    return 0;
}