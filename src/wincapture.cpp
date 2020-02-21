// Copyright (C) Microsoft Corporation. All rights reserved.
#include "wincapture.h"
using namespace winrt;
using namespace winrt::Windows::Foundation;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Devices::Enumeration;
using namespace winrt::Windows::System::Threading;
#define _INFO printf
#define _WARN printf
#define _ERROR printf
#define  LOG_TRANSFORMS() \
{\
int i = 0;\
                while (MF_E_INVALIDINDEX != spSourceReader.try_as<IMFSourceReaderEx>()->GetTransformForStream(MF_SOURCE_READER_FIRST_VIDEO_STREAM, i++, &guid, spTransform.put()))\
                {\
                    _INFO("\nTranform %d: %x-%x-%x-%x%x%x%x", i - 1, guid.Data1, guid.Data2, guid.Data3, guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3]);\
                }\
                _INFO("\nNumber of tranforms in Chain: %d", i - 1);\
}

namespace ros_win_camera
{
    WindowsMFCapture::WindowsMFCapture(bool isDevice, const winrt::hstring& link, bool isController /*=true*/)
        :m_nRefCount(1),
        m_u32SourceReaderFlags(0),
        m_u32Height(480),
        m_u32Width(640),
        m_bStreamingStarted(false),
        m_bIsController(isController)
    {
        InitializeCriticalSection(&m_critsec);
        m_configEventTokenList = single_threaded_vector<winrt::event_token>();

        if (isDevice)
        {
            InitCaptureWithDevice(link);
        }
        else
        {
            InitCaptureWithUrl(link);
        }
    }
    winrt::event_token  WindowsMFCapture::AddSampleHandler(winrt::delegate<winrt::hresult_error, winrt::hstring, IMFSample*> handler)
    {
        std::lock_guard g(m_apiGuardMutex);

        auto tok = m_captureCallbackEvent.add(handler);
        return tok;
    }
    void WindowsMFCapture::RemoveSampleHandler(winrt::event_token token)
    {
        std::lock_guard g(m_apiGuardMutex);
        m_captureCallbackEvent.remove(token);
    }
    winrt::Windows::Foundation::Collections::IVectorView<winrt::hstring> WindowsMFCapture::EnumerateCameraLinks(bool bEnumerateSensorCamera)
    {
        auto links = single_threaded_vector<winrt::hstring>();
        auto filteredDevices = DeviceInformation::FindAllAsync(DeviceClass::VideoCapture).get();
        if (!filteredDevices.Size())
        {
            throw_hresult(MF_E_NO_CAPTURE_DEVICES_AVAILABLE);
        }
        auto deviceIter = filteredDevices.First();
        while (deviceIter.HasCurrent())
        {
            auto device = deviceIter.Current();
            links.Append(device.Id());
            deviceIter.MoveNext();
        }
        return links.GetView();
    }

    void WindowsMFCapture::InitCaptureWithDevice(const winrt::hstring& cameraSymbolicLink)
    {

        winrt::com_ptr<IMFAttributes> spAttributes;
        winrt::com_ptr<IMFAttributes> spSRAttributes;

        winrt::com_ptr<IMFSensorGroup> spSensorGrp;
        winrt::com_ptr<IMFSensorDevice> spSensorDevice;

        check_hresult(MFCreateSensorGroup(cameraSymbolicLink.c_str(), spSensorGrp.put()));
        check_hresult(spSensorGrp->GetSensorDevice(0, spSensorDevice.put()));
        check_hresult(spSensorDevice->SetSensorDeviceMode(m_bIsController? MFSensorDeviceMode::MFSensorDeviceMode_Controller : MFSensorDeviceMode::MFSensorDeviceMode_Shared));
        check_hresult(spSensorGrp->CreateMediaSource(spMediaSource.put()));
        check_hresult(MFCreateAttributes(spSRAttributes.put(), 3));

        check_hresult(spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE));
        check_hresult(spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
        check_hresult(spSRAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this));

        check_hresult(MFCreateSourceReaderFromMediaSource(spMediaSource.get(), spSRAttributes.get(), spSourceReader.put()));
    }

    void WindowsMFCapture::StopStreaming()
    {
        std::lock_guard g(m_apiGuardMutex);
        if (!m_bStreamingStarted) return;

        std::mutex completionMutex;
        completionMutex.lock();

        EnterCriticalSection(&m_critsec);
        m_configEventTokenList.Append(m_configEvent.add([&]()
        {
            m_bStreamingStarted = false;
            check_hresult(spSourceReader->SetStreamSelection((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, FALSE));
            completionMutex.unlock();
        }));
        LeaveCriticalSection(&m_critsec);
        check_hresult(spSourceReader->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM));

        // wait for the config-stop event to complete
        completionMutex.lock();
        _INFO("\nStopped streaming complete!\n");
        m_captureCallbackEvent(hresult_error(MF_E_END_OF_STREAM), L"Sample Stopped", nullptr);
    }

    void WindowsMFCapture::StartStreaming()
    {
        std::lock_guard g(m_apiGuardMutex);
        if (m_bStreamingStarted) return;

        winrt::com_ptr <IMFMediaType> spMT;
        GUID subtype;

        check_hresult(spSourceReader->SetStreamSelection((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE));
        check_hresult(spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, m_u32SourceReaderFlags, NULL, NULL, NULL, NULL));
        m_bStreamingStarted = true;
    }

    void WindowsMFCapture::InitCaptureWithUrl(const winrt::hstring& url)
    {
        winrt::com_ptr<IMFAttributes> spSRAttributes;
        GUID subtype;

        check_hresult(CoInitialize(NULL));
        check_hresult(MFStartup(MF_VERSION));
        
        check_hresult(MFCreateAttributes(spSRAttributes.put(), 3));
        check_hresult(spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE));
        check_hresult(spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
        check_hresult(spSRAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this));
        check_hresult(MFCreateSourceReaderFromURL(url.c_str(), spSRAttributes.get(), spSourceReader.put()));
        m_u32Width = 640;
        m_u32Height = 480;
    }

    HRESULT WindowsMFCapture::OnReadSample(
        HRESULT hrStatus,
        DWORD /* dwStreamIndex */,
        DWORD dwStreamFlags,
        LONGLONG llTimestamp,
        IMFSample* pSample      // Can be NULL
    )
    {
        std::lock_guard g(m_sampleHandlerMutex);
        try
        {
            if (SUCCEEDED(hrStatus))
            {
                if (!pSample && !dwStreamFlags)
                {
                    //Drain completed
                    winrt::com_ptr <IMFMediaType> spMT;
                    GUID subtype;
                    m_u32SourceReaderFlags = 0;
                    check_hresult(spSourceReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMT.put()));

                    check_hresult(spMT->GetGUID(MF_MT_SUBTYPE, &subtype));
                    if (!IsEqualGUID(subtype, MFVideoFormat_ARGB32))
                    {
                        check_hresult(spMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));
                        check_hresult(spSourceReader->SetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMT.get()));
                    }
                    check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));

                    int i = 0;
                    GUID guid;
                    winrt::com_ptr<IMFTransform> spTransform;
                    while (MF_E_INVALIDINDEX != spSourceReader.try_as<IMFSourceReaderEx>()->GetTransformForStream(MF_SOURCE_READER_FIRST_VIDEO_STREAM, i++, &guid, spTransform.put()))
                    {
                        _INFO("Tranform %d: %x-%x-%x-%x%x%x%x", i - 1, guid.Data1, guid.Data2, guid.Data3, guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3]);
                    }
                    _INFO("Aftermath:Number of tranforms in Chain: %d", i - 1);

                }
            }
            else
            {
                throw_hresult(hrStatus);
            }
            if (MF_SOURCE_READERF_NATIVEMEDIATYPECHANGED & dwStreamFlags)
            {
                winrt::com_ptr <IMFMediaType> spMT;
                check_hresult(spSourceReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMT.put()));
                check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
                _INFO("\nNative Type changed: %dx%d", m_u32Width, m_u32Height);
            }
            if (MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED & dwStreamFlags)
            {
                winrt::com_ptr <IMFMediaType> spMT;
                check_hresult(spSourceReader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, spMT.put()));
                check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
                _INFO("\nCurrent Type changed %dx%d", m_u32Width, m_u32Height);
                GUID guid;
                winrt::com_ptr<IMFTransform> spTransform;
                LOG_TRANSFORMS();

                m_u32SourceReaderFlags = MF_SOURCE_READER_CONTROLF_DRAIN;

            }
            if (MF_SOURCE_READERF_ENDOFSTREAM & dwStreamFlags)
            {
                ////TODO: need to handle EOS
                m_captureCallbackEvent(hresult_error(MF_E_END_OF_STREAM), L"End Of stream",nullptr);
            }
        }
        catch (hresult_error const& ex)
        {
            _ERROR("%x:%s", (unsigned int)ex.code(), winrt::to_string(ex.message()).c_str());
            m_captureCallbackEvent(ex, L":Trying to read sample in callback", nullptr);
        }
        EnterCriticalSection(&m_critsec);
        m_configEvent();
        for (auto token : m_configEventTokenList)
        {
            m_configEvent.remove(token);
        }
        LeaveCriticalSection(&m_critsec);
        if (m_bStreamingStarted)
        {
            if (SUCCEEDED(hrStatus) && pSample)
            {
                m_captureCallbackEvent(hresult_error(S_OK), L"Sample received", pSample);
            }

            HRESULT hr = spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, m_u32SourceReaderFlags, NULL, NULL, NULL, NULL);
            if (hr == MF_E_NOTACCEPTING)
            {
                m_captureCallbackEvent(hresult_error(S_OK), L"Flush in progress", nullptr);
            }
            else if (FAILED(hr))
            {
                hresult_error ex(hr);
                _ERROR("%x:%s", (unsigned int)ex.code(), winrt::to_string(ex.message()).c_str());
                m_captureCallbackEvent(ex, L":Trying to read sample in callback", nullptr);
            }
        }
        //else
        //{
        //    //As we are in the sample callback and m_bStreamingStarted is false, that means streaming must have stopped
        //    m_captureCallbackEvent(hresult_error(MF_E_END_OF_STREAM), L"Sample Stopped", nullptr);
        //    
        //}

        return S_OK;
    }
    bool WindowsMFCapture::FindMatchingMediaType(IMFMediaType** ppMediaType, int32_t width/*=0*/, int32_t height/*=0*/, float frameRate/*=0*/, GUID preferredVideoSubType/*=GUID_NULL*/)
    {
        winrt::com_ptr<IMFMediaType> spMediaType;
        int idx = 0;
        UINT32 FRNum, FRDen;
        bool bMatch = true;
        GUID subType = GUID_NULL;
        check_pointer(ppMediaType);
        while (S_OK == spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, idx, spMediaType.put()))
        {
            check_hresult(MFGetAttributeSize(spMediaType.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
            check_hresult(MFGetAttributeRatio(spMediaType.get(), MF_MT_FRAME_RATE, &FRNum, &FRDen));
            check_hresult(spMediaType->GetGUID(MF_MT_SUBTYPE,&subType));
            float rate = ((float)FRNum / (float)FRDen);
            //_INFO("\nGot supported resolution :%dx%d@%d", m_u32Width, m_u32Height, (int)rate);
            bMatch = ((height == 0) || (width == 0) || ((m_u32Width == width) && (height == m_u32Height)));
            bMatch = bMatch && (((int)rate == (int)frameRate) || (frameRate == 0));
            bMatch = bMatch &&  (IsEqualGUID(preferredVideoSubType, subType) || IsEqualGUID(preferredVideoSubType, GUID_NULL));
            if (bMatch)
            {
                break;
            }
            idx++;
        }
        spMediaType.copy_to(ppMediaType);
        return bMatch;

    }
    bool WindowsMFCapture::ChangeCaptureConfig(int32_t width, int32_t height, float frameRate, GUID preferredVideoSubType, bool bForceConversion /*= false*/)
    {
        std::lock_guard g(m_apiGuardMutex);
        bool bStatus = true;
        slim_mutex completionMuxtex;
        completionMuxtex.lock();
        auto configHandler = [&]()
        {
            HRESULT hr = S_OK;
            winrt::com_ptr<IMFMediaType> spMediaType;
            UINT32 FRNum, FRDen;
            int idx = 0;
            _INFO("\nSetting resolution :%dx%d@%f", width, height, frameRate);
            if (!m_bIsController)
            {
                _WARN("Instance is not in controller mode. Setting up a conversion");
                HRESULT hr = spSourceReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMediaType.put());

                check_hresult(MFSetAttributeSize(spMediaType.get(), MF_MT_FRAME_SIZE, width, height));
                check_hresult(MFSetAttributeRatio(spMediaType.get(), MF_MT_FRAME_RATE, (UINT32)(frameRate*100), 100 ));
                check_hresult(spMediaType->SetGUID(MF_MT_SUBTYPE, preferredVideoSubType));
                check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMediaType.get()));
                m_u32Width = width;
                m_u32Height = height;

            }
            if (!FindMatchingMediaType(spMediaType.put(), width, height, frameRate, preferredVideoSubType))
            {
                if (bForceConversion)
                {
                    _WARN("No matching resolution and subtype supported by source. Setting up a conversion");
                    if (!FindMatchingMediaType(spMediaType.put(), width, height, frameRate, GUID_NULL))
                    {
                        if (!FindMatchingMediaType(spMediaType.put(), width, height, 0, GUID_NULL))
                        {
                            bStatus = false;
                        }
                    }
                    if (bStatus)
                    {
                        DWORD flags;
                        check_hresult(spSourceReader.as<IMFSourceReaderEx>()->SetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, spMediaType.get(), &flags));
                        check_hresult(spMediaType->SetGUID(MF_MT_SUBTYPE, preferredVideoSubType));
                        check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMediaType.get()));
                        m_u32Width = width;
                        m_u32Height = height;
                    }
                }
                else
                {
                    bStatus = false;
                }
            }
            else
            {
                DWORD flags = 0;
                GUID subtype;
                HRESULT hr = spSourceReader.as<IMFSourceReaderEx>()->SetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, spMediaType.get(), &flags);

                if (SUCCEEDED(hr))
                {
                    _INFO("Native res set");
                    m_u32Width = width;
                    m_u32Height = height;
                }
                else
                {
                    _WARN("Couldn't set native res: %x", hr);
                }
            }
            completionMuxtex.unlock();
        };
        if (m_bStreamingStarted)
        {
            EnterCriticalSection(&m_critsec);
             // post a delegate to be processed by the sample processing thread, so the order of mediatype change is maintained
            m_configEventTokenList.Append(m_configEvent.add(configHandler));
            LeaveCriticalSection(&m_critsec);
            check_hresult(spSourceReader->Flush(MF_SOURCE_READER_FIRST_VIDEO_STREAM));
            completionMuxtex.lock();
            check_hresult(spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, m_u32SourceReaderFlags, NULL, NULL, NULL, NULL));
        }
        else
        {
            configHandler();
        }
        return bStatus;
    }
}