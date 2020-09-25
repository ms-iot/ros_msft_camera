// Copyright (C) Microsoft Corporation. All rights reserved.
#include "wincapture.h"
#include "winrospublisher.h"
#include <ros/ros.h>
#include <string>

#include <EventToken.h>
#include <windows.h>
#include <wincrypt.h>
#include <windows.foundation.h>
#include <windows.foundation.collections.h>
#include <winrt\base.h>
#include <winrt\Windows.Media.h>
#include <winrt\Windows.Foundation.h>
#include "RTSPServerControl.h"
#include "RTPMediaStreamer.h"
using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_msft_camera;
constexpr int32_t PUBLISHER_QUEUE_SIZE = 4;
constexpr int32_t DEFAULT_WIDTH = 640;
constexpr int32_t DEFAULT_HEIGHT = 480;
constexpr float DEFAULT_FRAMERATE = 30.0;
constexpr int32_t DEFAULT_RTSP_PORT = 8554;
auto videoFormat = MFVideoFormat_ARGB32;

IMFSinkWriter* InitSinkWriter(IMFMediaSink* pMediaSink, uint32_t width, uint32_t height, float framerate, GUID subType)
{
    com_ptr<IMFSinkWriter> spSinkWriter;
    com_ptr<IMFAttributes> spSWAttributes;
    winrt::com_ptr<IMFMediaType> spInType;
    std::map<hstring, GUID> subtypeMap =
    {
        {L"NV12", MFVideoFormat_NV12},
        {L"YUY2", MFVideoFormat_YUY2},
        {L"IYUV", MFVideoFormat_IYUV},
        {L"ARGB32", MFVideoFormat_ARGB32}
    };

    check_hresult(MFCreateMediaType(spInType.put()));
    check_hresult(MFSetAttributeSize(spInType.get(), MF_MT_FRAME_SIZE, width, height));
    check_hresult(MFSetAttributeRatio(spInType.get(), MF_MT_FRAME_RATE, (uint32_t)(framerate * 100), 100));

    check_hresult(spInType->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spInType->SetGUID(MF_MT_SUBTYPE, subType));

    check_hresult(MFCreateAttributes(spSWAttributes.put(), 2));
    check_hresult(spSWAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    HRESULT hr = S_OK;
    BOOL bEnableHWTransforms = FALSE;
    do
    {
        spSinkWriter = nullptr;
        check_hresult(spSWAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, bEnableHWTransforms));
        check_hresult(MFCreateSinkWriterFromMediaSink(pMediaSink, spSWAttributes.get(), spSinkWriter.put()));
        hr = spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr);
        bEnableHWTransforms = !bEnableHWTransforms;
    } while (hr == MF_E_TOPO_CODEC_NOT_FOUND);
    check_hresult(hr);
    check_hresult(spSinkWriter->BeginWriting());

    return spSinkWriter.detach();
}

IRTSPAuthProvider* SetupRTSPAuthentication(ros::NodeHandle* pPrivateNode)
{
    winrt::com_ptr<IRTSPAuthProvider> spAuthProvider;
    std::map<std::string, std::string> credsAdd;
    std::vector<std::string> credsRemove;
    pPrivateNode->param("rtsp_AddCredentials", credsAdd, credsAdd);
    pPrivateNode->param("rtsp_RemoveCredentials", credsRemove, credsRemove);
    winrt::check_hresult(GetAuthProviderInstance(AuthType::Digest, winrt::to_hstring(pPrivateNode->getNamespace()).c_str(), spAuthProvider.put()));
    auto spCredStore = spAuthProvider.as<IRTSPAuthProviderCredStore>();
    for (auto&& user : credsRemove)
    {
        winrt::check_hresult(spCredStore->RemoveUser(winrt::to_hstring(user).c_str()));
    }
    for (auto&& cred : credsAdd)
    {
        auto user = winrt::to_hstring(cred.first);
        auto pass = winrt::to_hstring(cred.second);
        winrt::check_hresult(spCredStore->AddUser(user.c_str(), pass.c_str()));
    }

    return spAuthProvider.detach();
}
IMFSinkWriter* SetupRTSPServer(uint32_t width, uint32_t height, float framerate, GUID inSubType, ros::NodeHandle* pPrivateNode, IRTSPServerControl** ppServerControl)
{
    winrt::com_ptr<IMFMediaType> spOutMT;
    winrt::com_ptr<IMFMediaSink> spMediaSink;
    winrt::com_ptr<IRTSPServerControl> spServerControl;
    PCCERT_CONTEXT pCert = nullptr;
    int rtspPort = DEFAULT_RTSP_PORT;
    std::string secureCertFile = "";
    int bitRate = width * 1000;

    pPrivateNode->param("rtsp_port", rtspPort, rtspPort);
    pPrivateNode->param("rtsps_cert_filepath", secureCertFile, secureCertFile);
    pPrivateNode->param("rtp_bitrate", bitRate, bitRate);

    MFCreateMediaType(spOutMT.put());
    check_hresult(MFSetAttributeSize(spOutMT.get(), MF_MT_FRAME_SIZE, width, height));
    check_hresult(MFSetAttributeRatio(spOutMT.get(), MF_MT_FRAME_RATE, (uint32_t)(framerate * 100), 100));
    winrt::check_hresult(spOutMT->SetUINT32(MF_MT_AVG_BITRATE, bitRate));

    winrt::check_hresult(spOutMT->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive));
    check_hresult(spOutMT->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
    check_hresult(spOutMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_H264));

    IMFMediaType* mts[1];
    mts[0] = spOutMT.get();
    winrt::check_hresult(CreateRTPMediaSink(mts, 1, spMediaSink.put()));
    auto pSinkWriter = InitSinkWriter(spMediaSink.get(), width, height, framerate, inSubType);
    winrt::RTSPSuffixSinkMap rtspMap;
    rtspMap.Insert(L"/", spMediaSink.as<winrt::Windows::Foundation::IInspectable>());
    if (!secureCertFile.empty())
    {

        FILE* fp = fopen(secureCertFile.c_str(), "rb");
        fseek(fp, 0, SEEK_END);
        auto sz = ftell(fp);
        fseek(fp, 0, SEEK_SET);
        std::unique_ptr<BYTE[]>  certData = std::make_unique<BYTE[]>(sz);
        if (fread(certData.get(), sz, 1, fp) > 0)
        {
            pCert = CertCreateCertificateContext(X509_ASN_ENCODING, certData.get(), sz);
        }
    }
    winrt::com_ptr<IRTSPAuthProvider> spAuthProvider;
    spAuthProvider.attach(SetupRTSPAuthentication(pPrivateNode));
    winrt::check_hresult(CreateRTSPServer(rtspMap.as<ABI::RTSPSuffixSinkMap>().get(), rtspPort, (pCert != nullptr), spAuthProvider.get(),&pCert, pCert?1:0, spServerControl.put()));

    *ppServerControl = spServerControl.detach();
    return pSinkWriter;
}

std::string FormatUrl(std::string cameraInfoUrl)
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
    return cameraInfoUrl;
}

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
        winrt::com_ptr< ros_msft_camera::WindowsMFCapture> camera;
        winrt::event_token imagePubHandlerToken;
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
                // no source path is specified; we default to first enumerated camera
                videoSourcePath = winrt::to_string(ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(0));
            }
        }
        else if (videoSourcePath == "Interactive")
        {
            int i = 1;
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
                videoSourcePath = winrt::to_string(ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(i - 1));
            }
            else
            {
                ROS_ERROR("\nINTERATIVE mode for camera selection needs logger levels set atleast to Level::Info");
            }
        }
        std::shared_ptr<camera_info_manager::CameraInfoManager> spCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>(privateNode, frame_id, "");
        if (privateNode.getParam("camera_info_url", cameraInfoUrl))
        {
            cameraInfoUrl = FormatUrl(cameraInfoUrl);
            if (spCameraInfoManager->validateURL(cameraInfoUrl))
            {
                ROS_INFO("validated Camera info url: %s", cameraInfoUrl.c_str());
                spCameraInfoManager->loadCameraInfo(cameraInfoUrl);
            }
        }

        winrt::com_ptr<IMFSinkWriter> spSinkWriter;
        winrt::com_ptr<IRTSPServerControl> spRTSPServer;
        auto rawPublisher = new WinRosPublisherImageRaw(privateNode, "image_raw", queueSize, frame_id, spCameraInfoManager.get());

        bool resChangeInProgress = false;
        winrt::delegate<winrt::hresult_error, winrt::hstring, IMFSample*> rosImagePubHandler = [&](winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
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
                    spSinkWriter? winrt::check_hresult(spSinkWriter->WriteSample(0, pSample)) : 0;
                }
            }
            else
            {
                if ((HRESULT)ex.code().value == MF_E_HW_MFT_FAILED_START_STREAMING)
                {
                    ROS_WARN("WindowsMFCapture streaming failed. Retrying with shared mode...\n");
                    //  re-try with sharing mode
                    camera->RemoveSampleHandler(imagePubHandlerToken);
                    camera = nullptr;
                    camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), false));

                    camera->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
                    camera->StartStreaming();
                    imagePubHandlerToken = camera->AddSampleHandler(rosImagePubHandler);

                    ROS_WARN("WindowsMFCapture is in shared mode. Config paramters may not be applied \n");
                }
                if ((HRESULT)ex.code().value == MF_E_END_OF_STREAM)
                {
                    ROS_INFO("\nEOS");
                }
                eventFinish.notify_all();
            }
        };

        camera.attach(WindowsMFCapture::CreateInstance(isDevice, winrt::to_hstring(videoSourcePath), true));
        camera->ChangeCaptureConfig(Width, Height, frameRate, videoFormat, true);
        if (privateNode.hasParam("rtsp_port"))
        {
            spSinkWriter.attach(SetupRTSPServer(Width, Height, frameRate, videoFormat, &privateNode, spRTSPServer.put()));
            winrt::LogHandler rtsplogger([](winrt::hresult hr, winrt::hstring msg)
                {
                    if (SUCCEEDED(hr.value))
                    {
                        ROS_INFO("%s", winrt::to_string(msg).c_str());
                    }
                    else
                    {
                        ROS_ERROR("%x:%s\n", hr.value, winrt::to_string(msg).c_str());
                    }
                });
            for (int l = (int)LoggerType::ERRORS; l < (int)LoggerType::LOGGER_MAX; l++)
            {
                EventRegistrationToken t;
                spRTSPServer->AddLogHandler((LoggerType)l, rtsplogger.as<ABI::LogHandler>().get(), &t);
            }
            spRTSPServer->StartServer();
        }
        camera->StartStreaming();
        imagePubHandlerToken = camera->AddSampleHandler(rosImagePubHandler);
        

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
    catch (...)
    {
        ROS_ERROR(winrt::to_string(L"\nError:" + winrt::to_hstring(winrt::to_hresult())).c_str());
    }
}
