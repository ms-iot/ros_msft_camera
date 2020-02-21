#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>
#include <nodelet/nodelet.h>

namespace ros_win_camera
{
    const int32_t PUBLISHER_BUFFER_SIZE = 4;

    class WinCameraNodelet : public nodelet::Nodelet
    {

    public:
        WinCameraNodelet()
        {
        }
        ~WinCameraNodelet()
        {
            waitForFinish.lock();
        }

    private:
        virtual void onInit()//int main(int argc, char** argv)
        {
            //ros::init(argc, argv, "win_camera");
            //ros::NodeHandle private_node("~");
            auto privateNode = getPrivateNodeHandle();
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
            auto handler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
            {
                if (pSample)
                {
                    //castd::cout << "Received SAmple\n";
                    rawPublisher.OnSample(pSample, Width, Height);
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
                    std::cout << "Received SAmple handller1\n";

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
            if (!camera->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_MJPG))
            {
                camera->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_ARGB32, true);
                camera->AddSampleHandler(handler);
            }
            else
            {
                camera1.attach(new ros_win_camera::WindowsMFCapture(isDevice, winrt::to_hstring(videoSourcePath), false));
                camera1->ChangeCaptureConfig(Width, Height, frameRate, MFVideoFormat_ARGB32);
                camera->AddSampleHandler(handler1);
                camera1->AddSampleHandler(handler);
            }

            //Sleep(3000);
            //camera->StopStreaming();
        }
        winrt::slim_mutex waitForFinish;
    };
}