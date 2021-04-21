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
#ifdef ENABLE_RTSP
#include "RTSPServerControl.h"
#include "RTPMediaStreamer.h"
#endif
using namespace winrt::Windows::System::Threading;
using namespace winrt::Windows::Foundation;
using namespace ros_msft_camera;
constexpr int32_t PUBLISHER_QUEUE_SIZE = 4;
constexpr int32_t DEFAULT_WIDTH = 640;
constexpr int32_t DEFAULT_HEIGHT = 480;
constexpr float DEFAULT_FRAMERATE = 30.0;
constexpr int32_t DEFAULT_RTSP_PORT = 8554;
const GUID g_rosPrefferdVideoFormat = MFVideoFormat_ARGB32;
const GUID g_rtspPrefferdVideoFormat = MFVideoFormat_NV12;

class MsftCameraNode : ros::NodeHandle
{
public:
    MsftCameraNode(std::string ns)
        : ros::NodeHandle(ns)
    {
        param("frame_rate", m_frameRate, m_frameRate);
        param("pub_queue_size", m_queueSize, m_queueSize);
        param("image_width", m_width, DEFAULT_WIDTH);
        param("image_height", m_height, DEFAULT_HEIGHT);
        param("frame_id", m_frameId, std::string("camera"));
        getParam("videoDeviceId", m_videoSourcePath);
        if (m_videoSourcePath.empty())
        {
            getParam("videoUrl", m_videoSourcePath);
            if (!m_videoSourcePath.empty())
            {
                m_isDevice = false;
            }
            else
            {
                // no source path is specified; we default to first enumerated camera
                m_videoSourcePath = winrt::to_string(ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false).GetAt(0));
            }
        }

        else if (m_videoSourcePath == "Interactive")
        {
            int i = 1;
            std::map< std::string, ros::console::levels::Level> logger;
            ros::console::get_loggers(logger);
            if (logger[ROSCONSOLE_DEFAULT_NAME] == ros::console::levels::Level::Info)
            {
                auto links = ros_msft_camera::WindowsMFCapture::EnumerateCameraLinks(false);
                for (auto sym : links)
                {
                    ROS_INFO("%d. %s", i++, sym.c_str());
                }
                do
                {
                    ROS_INFO("\nYour choice [1 to %d]: ", links.Size());
                    std::cin >> i;
                } while ((i < 1) || (i > links.Size()));
                m_videoSourcePath = winrt::to_string(links.GetAt(i - 1));
            }
            else
            {
                ROS_ERROR("\nINTERACTIVE mode for camera selection needs logger levels set atleast to Level::Info");
            }
        }

        m_spCameraInfoManager = std::make_shared<camera_info_manager::CameraInfoManager>((ros::NodeHandle) * this, m_frameId, "");
        if (getParam("camera_info_url", m_cameraInfoUrl))
        {
            m_cameraInfoUrl = FormatUrl(m_cameraInfoUrl);
            if (m_spCameraInfoManager->validateURL(m_cameraInfoUrl))
            {
                ROS_INFO("validated Camera info url: %s", m_cameraInfoUrl.c_str());
                m_spCameraInfoManager->loadCameraInfo(m_cameraInfoUrl);
            }
        }

        m_spRawImagePublisher = std::make_unique<WinRosPublisherImageRaw>(*((ros::NodeHandle *)this), "image_raw", m_queueSize, m_frameId, m_spCameraInfoManager);
        ConfigureMFCapture(true);
    }

    virtual ~MsftCameraNode()
    {
        Stop();
    }
    void Start()
    {
        m_spWinMFCaptureRos->StartStreaming();
        m_imagePubHandlerToken = m_spWinMFCaptureRos->AddSampleHandler({ this, &MsftCameraNode::ImagePubHandler });

#ifdef TEST_SETCAMERAINFO
        Sleep(10000);
        auto info = m_spCameraInfoManager->getCameraInfo();
        info.height = 720;
        info.width = 400;
        m_spCameraInfoManager->setCameraInfo(info);
#endif //#ifdef TEST_SETCAMERAINFO

#ifdef ENABLE_RTSP
        if (m_spRTSPServer && m_spWinMFCaptureRtsp)
        {
            m_spWinMFCaptureRtsp->StartStreaming();
            m_rtspHandlerToken = m_spWinMFCaptureRtsp->AddSampleHandler({ this, &MsftCameraNode::rtpHandler });
            m_spRTSPServer->StartServer();
        }
#endif
    }

    void Stop()
    {
        std::mutex mutexFinish;
        std::unique_lock<std::mutex> lockFinish(mutexFinish);
        ThreadPool::RunAsync([&](IAsyncAction)
            {
                m_spWinMFCaptureRos? m_spWinMFCaptureRos->StopStreaming() : 0;
                m_spWinMFCaptureRtsp ? m_spWinMFCaptureRtsp->StopStreaming() : 0;
            });

        if (m_imagePubHandlerToken)
        {
            m_eventFinish.wait(lockFinish);
            m_spWinMFCaptureRos->RemoveSampleHandler(m_imagePubHandlerToken);
            m_imagePubHandlerToken.value = 0;
        }
#ifdef ENABLE_RTSP
        if (m_spRTSPServer)
        {
            m_spRTSPServer->StopServer();
            if (m_rtspHandlerToken)
            {
                m_spWinMFCaptureRtsp->RemoveSampleHandler(m_rtspHandlerToken);
                m_rtspHandlerToken.value = 0;
            }
        }
#endif
    }

private:
    std::string FormatUrl(std::string m_cameraInfoUrl)
    {
        auto pos = m_cameraInfoUrl.find("file:");
        if (pos != std::string::npos)
        {
            // this block of code is required because there is a bug in camerainfo manager url based api 
            // when handling file:// prefix url with local paths like "e:\folder"
            // We convert such paths to unc path.

            pos += 5; // length of "file:"
            int slashCnt = 0;
            while (m_cameraInfoUrl[++pos] == '/') slashCnt++;
            if (m_cameraInfoUrl[pos] == '\\' && m_cameraInfoUrl[pos + 1] == '\\')
            {
                // unc path that needs conversion
                auto path = m_cameraInfoUrl.substr(pos + 2);
                m_cameraInfoUrl = std::string("file:////") + path;
            }
            else if (m_cameraInfoUrl[pos + 1] == ':')
            {
                //full drive path
                auto path = m_cameraInfoUrl.substr(pos);
                path[1] = '$'; // replace ':' with '$' to convert to unc path
                m_cameraInfoUrl = std::string("file:////127.0.0.1\\") + path;
            }
            else if (slashCnt < 4) // if slashCnt >= 4 it is probably is an acceptable unc path
            {
                ROS_ERROR("camera info url for file must be a fully qualified drive path or unc path");
            }
        }
        return m_cameraInfoUrl;
    }

    void ConfigureMFCapture(bool isController)
    {
        m_spWinMFCaptureRos = nullptr;
        m_spWinMFCaptureRtsp = nullptr;
#ifdef ENABLE_RTSP
        m_spRTSPServer = nullptr;
#endif
        auto winMFcap = WindowsMFCapture::CreateInstance(m_isDevice, winrt::to_hstring(m_videoSourcePath), isController);
        m_spWinMFCaptureRtsp.attach(winMFcap);
        if (winMFcap->ChangeCaptureConfig(m_width, m_height, m_frameRate, g_rosPrefferdVideoFormat, !isController))
        {
#ifdef ENABLE_RTSP
            rtspInVideoFormat = g_rosPrefferdVideoFormat;
#endif
            m_spWinMFCaptureRos.copy_from(winMFcap);
        }
        else
        {
#ifdef ENABLE_RTSP
            if (hasParam("rtsp_port"))
            {
                winMFcap->ChangeCaptureConfig(m_width, m_height, m_frameRate, g_rtspPrefferdVideoFormat, true);
                rtspInVideoFormat = g_rtspPrefferdVideoFormat;
                m_spWinMFCaptureRos.attach(WindowsMFCapture::CreateInstance(m_isDevice, winrt::to_hstring(m_videoSourcePath), false));
            }
            else
#endif
            {
                m_spWinMFCaptureRos.attach(winMFcap);
            }
            m_spWinMFCaptureRos->ChangeCaptureConfig(m_width, m_height, m_frameRate, g_rosPrefferdVideoFormat, true);
        }

#ifdef ENABLE_RTSP
        /*RTSP*/
        if (hasParam("rtsp_port"))
        {
            SetupRTSPServer(m_width, m_height, m_frameRate, rtspInVideoFormat, this);
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
                ::EventRegistrationToken t;
                m_spRTSPServer->AddLogHandler((LoggerType)l, rtsplogger.as<ABI::LogHandler>().get(), &t);
            }
        }
#endif
    }

    void ImagePubHandler(winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
    {
        if (pSample)
        {
            auto info = m_spCameraInfoManager->getCameraInfo();
            if (((info.height != m_height) || (info.width != m_width)) && info.height && info.width && (!m_configChangeInProgress))
            {
                m_configChangeInProgress = true;
                ThreadPool::RunAsync([&](IAsyncAction)
                    {
                        auto info = m_spCameraInfoManager->getCameraInfo();
                        if (m_spWinMFCaptureRos->ChangeCaptureConfig(info.width, info.height, m_frameRate, g_rosPrefferdVideoFormat, true))
                        {
                            m_height = info.height;
                            m_width = info.width;
                            m_configChangeInProgress = false;
                        }
                        else
                        {
                            ROS_WARN("Setting resolution  failed. Use rescale_camera_info param for rescaling");
                        }
                    });
            }
            else
            {
                m_spRawImagePublisher->OnSample(pSample, (UINT32)m_width, (UINT32)m_height);
            }
        }
        else
        {
            HandleStreamingErrors(ex.code());
        }
    }

    void HandleStreamingErrors(HRESULT hr)
    {
        if ((hr == MF_E_HW_MFT_FAILED_START_STREAMING)
            || (hr == winrt::impl::hresult_from_win32(ERROR_SHARING_VIOLATION))
            )
        {
            if (!m_configChangeInProgress)
            {
                //  re-try with sharing mode
                ROS_WARN("WindowsMFCapture streaming failed. Retrying with shared mode...\n");
                m_configChangeInProgress = true;
                ThreadPool::RunAsync([this](IAsyncAction)
                    {
                        Stop();
                        ConfigureMFCapture(false);
                        Start();
                        m_configChangeInProgress = false;
                        ROS_WARN("WindowsMFCapture is in shared mode. Config paramters may not be applied \n");
                    });
            }
            else
            {
                ROS_WARN("Config Change is in progress.. \n");
                return;
            }
        }
        if (hr == MF_E_END_OF_STREAM)
        {
            ROS_INFO("\nEOS");
        }
        m_eventFinish.notify_all();
    }

    std::string m_videoSourcePath = "";
    std::string m_frameId = "camera";
    std::string m_cameraInfoUrl = "";
    bool m_isDevice = true;
    float m_frameRate = DEFAULT_FRAMERATE;
    int32_t m_queueSize = PUBLISHER_QUEUE_SIZE;
    int32_t m_width = DEFAULT_WIDTH;
    int32_t m_height = DEFAULT_HEIGHT;
    winrt::com_ptr<ros_msft_camera::WindowsMFCapture> m_spWinMFCaptureRos, m_spWinMFCaptureRtsp;
    std::shared_ptr<camera_info_manager::CameraInfoManager> m_spCameraInfoManager;
    bool m_configChangeInProgress = false;
    std::unique_ptr<WinRosPublisherImageRaw> m_spRawImagePublisher;
    std::condition_variable m_eventFinish;
    winrt::event_token m_imagePubHandlerToken, m_rtspHandlerToken;

    /*RTSP*/
#ifdef ENABLE_RTSP
    void InitSinkWriter(IMFMediaSink* pMediaSink, uint32_t width, uint32_t height, float framerate, GUID subType)
    {
        com_ptr<IMFAttributes> spSWAttributes;
        winrt::com_ptr<IMFMediaType> spInType;

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
            m_spSinkWriter = nullptr;
            check_hresult(spSWAttributes->SetUINT32(MF_READWRITE_ENABLE_HARDWARE_TRANSFORMS, bEnableHWTransforms));
            check_hresult(MFCreateSinkWriterFromMediaSink(pMediaSink, spSWAttributes.get(), m_spSinkWriter.put()));
            hr = m_spSinkWriter->SetInputMediaType(0, spInType.get(), nullptr);
            bEnableHWTransforms = !bEnableHWTransforms;
        } while (hr == MF_E_TOPO_CODEC_NOT_FOUND);
        check_hresult(hr);
        check_hresult(m_spSinkWriter->BeginWriting());
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

    // sample test code to get localhost test certificate
    std::vector<PCCERT_CONTEXT> GetServerCertificate(LPCWSTR subject)
    {
        std::vector<PCCERT_CONTEXT> aCertContext;
        PCCERT_CONTEXT pCertContext = NULL;
        //-------------------------------------------------------
        // Open the My store, also called the personal store.
        // This call to CertOpenStore opens the Local_Machine My 
        // store as opposed to the Current_User's My store.

        auto hMyCertStore = CertOpenStore(CERT_STORE_PROV_SYSTEM,
            X509_ASN_ENCODING,
            0,
            CERT_SYSTEM_STORE_LOCAL_MACHINE,
            L"MY");

        if (hMyCertStore == NULL)
        {
            ROS_ERROR("Error opening MY store for server.\n");
            return aCertContext;
        }
        //-------------------------------------------------------
        // Search for a certificate with some specified
        // string in it. This example attempts to find
        // a certificate with the string "example server" in
        // its subject string. Substitute an appropriate string
        // to find a certificate for a specific user.
        do
        {
            pCertContext = CertFindCertificateInStore(hMyCertStore,
                X509_ASN_ENCODING,
                0,
                CERT_FIND_SUBJECT_STR_W,
                subject, // use appropriate subject name
                pCertContext
            );
            if (pCertContext)
            {
                aCertContext.push_back(CertDuplicateCertificateContext(pCertContext));
            }
        } while (pCertContext);

        if (aCertContext.empty())
        {
            ROS_ERROR("Error retrieving server certificate.");
        }

        if (hMyCertStore)
        {
            CertCloseStore(hMyCertStore, 0);
        }
        return aCertContext;
    }

    void SetupRTSPServer(uint32_t width, uint32_t height, float framerate, GUID inSubType, ros::NodeHandle* pPrivateNode)
    {
        winrt::com_ptr<IMFMediaType> spOutMT;
        winrt::com_ptr<IMFMediaSink> spMediaSink;
        std::vector<PCCERT_CONTEXT> pCert;
        int rtspPort = DEFAULT_RTSP_PORT;
        std::string secureCertSubject = "";
        int bitRate = width * 1000;

        pPrivateNode->param("rtsp_port", rtspPort, rtspPort);
        pPrivateNode->param("rtsps_cert_subject", secureCertSubject, secureCertSubject);
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
        InitSinkWriter(spMediaSink.get(), width, height, framerate, inSubType);
        winrt::RTSPSuffixSinkMap rtspMap;
        rtspMap.Insert(L"/", spMediaSink.as<winrt::Windows::Foundation::IInspectable>());
        if (!secureCertSubject.empty())
        {
            pCert = GetServerCertificate(winrt::to_hstring(secureCertSubject).c_str());
        }
        winrt::com_ptr<IRTSPAuthProvider> spAuthProvider;
        spAuthProvider.attach(SetupRTSPAuthentication(pPrivateNode));
        winrt::check_hresult(CreateRTSPServer(rtspMap.as<ABI::RTSPSuffixSinkMap>().get(), rtspPort, !pCert.empty(), spAuthProvider.get(), pCert.data(), pCert.size(), m_spRTSPServer.put()));

    }

    void rtpHandler(winrt::hresult_error ex, winrt::hstring msg, IMFSample* pSample)
    {
        if (pSample)
        {
            winrt::check_hresult(m_spSinkWriter->WriteSample(0, pSample));
        }
        else
        {
            // if both capture objects are same, then the ImagePubHandler 
            // (set on m_spWinMFCaptureRos) must have received the same error; Let ImagePubHandler handle this error.
            if (m_spWinMFCaptureRos != m_spWinMFCaptureRtsp)
            {
                HandleStreamingErrors(ex.code());
            }
        }
    }


    GUID rtspInVideoFormat = MFVideoFormat_NV12;
    winrt::com_ptr<IMFSinkWriter> m_spSinkWriter;
    winrt::com_ptr<IRTSPServerControl> m_spRTSPServer;
#endif //ENABLE_RTSP
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ros_msft_camera");
    MsftCameraNode cameraNode("~");
    cameraNode.Start();
    ros::spin();
    cameraNode.Stop();
}