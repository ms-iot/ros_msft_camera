// Copyright 
#include "capture.h"
#include <Memorybuffer.h>


#include <winrt\Windows.Media.Core.h>
#include <winrt\Windows.System.Threading.h>
#include <windows.graphics.imaging.interop.h>
#include <windows.graphics.imaging.h>

#include <sstream>
#include <string>

using namespace winrt;
using namespace winrt::Windows::Foundation::Collections;
using namespace winrt::Windows::Media::Capture;
using namespace winrt::Windows::Media::Capture::Frames;
using namespace winrt::Windows::Graphics::Imaging;
using namespace winrt::Windows::Devices::Enumeration;
using namespace winrt::Windows::Media::Core;
using namespace winrt::Windows::System::Threading;

namespace ros_win_camera
{

namespace enc = sensor_msgs::image_encodings;

WindowsCapture::WindowsCapture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : m_nodeHandle(node),
      m_imageTransport(m_nodeHandle),
      m_topicName(topic_name),
      m_i32BufferSize(buffer_size),
      m_frameID(frame_id),
      m_cameraInfoManager(m_nodeHandle, frame_id),
      m_nRefCount(1),
      m_u32SourceReaderFlags(0)
{
    InitializeCriticalSection(&m_critsec);
    m_configEventTokenList = single_threaded_vector<winrt::event_token>();
}

void WindowsCapture::loadCameraInfo()
{
  std::string url;
  if (m_nodeHandle.getParam("camera_info_url", url))
  {
    if (m_cameraInfoManager.validateURL(url))
    {
      m_cameraInfoManager.loadCameraInfo(url);
    }
  }

  m_bRescaleCameraInfo = m_nodeHandle.param<bool>("rescale_camera_info", false);

  //for (int i = 0;; ++i)
  //{
  //  int code = 0;
  //  double value = 0.0;
  //  std::stringstream stream;
  //  stream << "property_" << i << "_code";
  //  const std::string param_for_code = stream.str();
  //  stream.str("");
  //  stream << "property_" << i << "_value";
  //  const std::string param_for_value = stream.str();
  //  if (!node_.getParam(param_for_code, code) || !node_.getParam(param_for_value, value))
  //  {
  //    break;
  //  }
  //  /*if (!cap_.set(code, value))
  //  {
  //    ROS_ERROR_STREAM("Setting with code " << code << " and value " << value << " failed"
  //                                          << std::endl);
  //  }*/
  //}
}

void WindowsCapture::RescaleCameraInfo(int width, int height)
{
  double width_coeff = static_cast<double>(width) / m_cameraInfo.width;
  double height_coeff = static_cast<double>(height) / m_cameraInfo.height;
  m_cameraInfo.width = width;
  m_cameraInfo.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  m_cameraInfo.K[0] *= width_coeff;
  m_cameraInfo.K[2] *= width_coeff;
  m_cameraInfo.K[4] *= height_coeff;
  m_cameraInfo.K[5] *= height_coeff;

  m_cameraInfo.P[0] *= width_coeff;
  m_cameraInfo.P[2] *= width_coeff;
  m_cameraInfo.P[5] *= height_coeff;
  m_cameraInfo.P[6] *= height_coeff;
}

void WindowsCapture::Open(const std::string& device_path, winrt::delegate<winrt::hresult_error, winrt::hstring> handler)
{
    ROS_INFO("\nin cap open");

    winrt::com_ptr<IMFAttributes> spAttributes;
    winrt::com_ptr<IMFAttributes> spSRAttributes;

    m_captureCallbackEvent.add(handler);
    winrt::hstring cameraSymbolicLink;
    
    auto filteredDevices = DeviceInformation::FindAllAsync(DeviceClass::VideoCapture).get();
    if (!filteredDevices.Size())
    {
        throw_hresult(MF_E_NO_CAPTURE_DEVICES_AVAILABLE);
    }
    auto deviceIter = filteredDevices.First();
    if ((device_path.empty()) || (device_path.c_str() == nullptr))
    {
        cameraSymbolicLink = deviceIter.Current().Id();
    }
    else
    {
        ROS_INFO("\nLooking for: %s", device_path.c_str());
        while (deviceIter.HasCurrent())
        {
            auto device = deviceIter.Current();
            if (winrt::to_string(device.Id()).compare(device_path) == 0)
            {
                cameraSymbolicLink = device.Id();
                break;
            }
            deviceIter.MoveNext();
        }
    }

    ROS_INFO("\nOpening: %s", winrt::to_string(cameraSymbolicLink).c_str());

    winrt::com_ptr<IMFSensorGroup> spSensorGrp;
    winrt::com_ptr<IMFSensorDevice> spSensorDevice;
    
    check_hresult(MFCreateSensorGroup(cameraSymbolicLink.c_str(), spSensorGrp.put()));
    check_hresult(spSensorGrp->GetSensorDevice(0, spSensorDevice.put()));
    check_hresult(spSensorDevice->SetSensorDeviceMode(MFSensorDeviceMode::MFSensorDeviceMode_Shared));
    check_hresult(spSensorGrp->CreateMediaSource(spMediaSource.put()));
    check_hresult(MFCreateAttributes(spSRAttributes.put(), 3));
    check_hresult(spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE));
    check_hresult(spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    check_hresult(spSRAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this));

    check_hresult(MFCreateSourceReaderFromMediaSource(spMediaSource.get(), spSRAttributes.get(), spSourceReader.put()));
    StartStreaming();
}
void WindowsCapture::StartStreaming()
{
    winrt::com_ptr <IMFMediaType> spMT;
    GUID subtype;
    try
    {
        MFCreateMediaType(spMT.put());
        check_hresult(spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMT.put()));
        check_hresult(spMT->GetGUID(MF_MT_SUBTYPE, &subtype));
        if (!IsEqualGUID(subtype, MFVideoFormat_ARGB32))
        {
            check_hresult(MFSetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, m_u32Width, m_u32Height));
            check_hresult(spMT->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video));
            check_hresult(spMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));
            check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMT.get()));
        }
        check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
        check_hresult(spSourceReader->SetStreamSelection((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE));
    }
    catch (hresult_error const& ex)
    {
        std::wcout << L"\nError: " << std::hex << ex.code() << L":" << ex.message().c_str();
        m_captureCallbackEvent(ex, L"Trying to open camera");
    }

    m_cameraPublisher = m_imageTransport.advertiseCamera(m_topicName, m_i32BufferSize);
    m_MFSamplePublisher = m_nodeHandle.advertise<win_camera::MFSample>("IMFSample", 2);

    loadCameraInfo();
    ROS_INFO("\nCamera open Done");
    m_u32SourceReaderFlags = 0;
    check_hresult(spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, m_u32SourceReaderFlags, NULL, NULL, NULL, NULL));
}

void WindowsCapture::OpenFile(const std::string &file_path, winrt::delegate<winrt::hresult_error, winrt::hstring> handler)
{
    winrt::com_ptr<IMFAttributes> spSRAttributes;
    GUID subtype;

    check_hresult(CoInitialize(NULL));
    check_hresult(MFStartup(MF_VERSION));
    m_captureCallbackEvent.add(handler);
    check_hresult(MFCreateAttributes(spSRAttributes.put(), 3));
    check_hresult(spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE));
    check_hresult(spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE));
    check_hresult(spSRAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this));
    check_hresult(MFCreateSourceReaderFromURL(winrt::to_hstring(file_path).c_str(),spSRAttributes.get(),spSourceReader.put()));
    m_u32Width = 640;
    m_u32Height = 480;
    StartStreaming();
}

bool WindowsCapture::PublishSample(IMFSample *pSample)
{
    if (pSample)
    {
        winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
        winrt::com_ptr<IMF2DBuffer2> spMediaBuf2d;
        uint32_t littleEndian = 1;
        BYTE* pix;
        LONG Stride;
        ros::Time now = ros::Time::now();

        m_cameraInfo = m_cameraInfoManager.getCameraInfo();
        if (m_cameraInfo.height == 0 && m_cameraInfo.width == 0)
        {
            m_cameraInfo.height = m_u32Height;
            m_cameraInfo.width = m_u32Width;
        }
        else if (m_cameraInfo.height != m_u32Height || m_cameraInfo.width != m_u32Width)
        {
            if (m_bRescaleCameraInfo)
            {
                int old_width = m_cameraInfo.width;
                int old_height = m_cameraInfo.height;
                RescaleCameraInfo(m_u32Width, m_u32Height);
                ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                    old_width, old_height, m_u32Width, m_u32Height);
            }
            else
            {
                ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                    "Use rescale_camera_info param for rescaling",
                    m_cameraInfo.width, m_cameraInfo.height, m_u32Width, m_u32Height);
            }
        }
        m_cameraInfo.header.stamp = now;
        m_cameraInfo.header.frame_id = m_frameID;

        check_hresult(pSample->GetBufferByIndex(0, spMediaBuf.put()));
        spMediaBuf2d = spMediaBuf.as<IMF2DBuffer2>();

        sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();
        ros_image->header = m_cameraInfo.header;
        ros_image->height = m_u32Height;
        ros_image->width = m_u32Width;
        ros_image->encoding = enc::BGRA8;
        ros_image->is_bigendian = !*((uint8_t*)&littleEndian);// (boost::endian::order::native == boost::endian::order::big);

        check_hresult(spMediaBuf2d->Lock2D(&pix, &Stride));
        ros_image->step = Stride;
        size_t size = Stride * m_u32Height;
        ros_image->data.resize(size);

        //if (image.isContinuous()) //we have set mediatype to ARGB32 which is contiguous in camera pipeline
        {
            memcpy((char*)(&ros_image->data[0]), pix, size);
        }
        check_hresult(spMediaBuf2d->Unlock2D());
        m_cameraPublisher.publish(*ros_image, m_cameraInfo);

        win_camera::MFSamplePtr sampleMsg = boost::make_shared<win_camera::MFSample>();
        sampleMsg->header = m_cameraInfo.header;
        sampleMsg->pSample = (UINT64)pSample;
        m_MFSamplePublisher.publish(*sampleMsg);

        return true;
    }
    return false;
}


HRESULT WindowsCapture::OnReadSample(
    HRESULT hrStatus,
    DWORD /* dwStreamIndex */,
    DWORD dwStreamFlags,
    LONGLONG llTimestamp,
    IMFSample* pSample      // Can be NULL
)
{
    EnterCriticalSection(&m_critsec);
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
                    ROS_INFO("Tranform %d: %x-%x-%x-%x%x%x%x", i - 1, guid.Data1, guid.Data2, guid.Data3, guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3]);
                }
                ROS_INFO("Aftermath:Number of tranforms in Chain: %d", i - 1);

            }
        }
        else
        {
            //pSample = nullptr;
            throw_hresult(hrStatus);
            //return S_OK;
        }
        if (MF_SOURCE_READERF_NATIVEMEDIATYPECHANGED & dwStreamFlags)
        {
            winrt::com_ptr <IMFMediaType> spMT;
            check_hresult(spSourceReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMT.put()));
            check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
            ROS_INFO("\nNative Type changed: %dx%d", m_u32Width, m_u32Height);
        }
        if (MF_SOURCE_READERF_CURRENTMEDIATYPECHANGED & dwStreamFlags)
        {
            winrt::com_ptr <IMFMediaType> spMT;
            check_hresult(spSourceReader->GetCurrentMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, spMT.put()));
            check_hresult(MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
            ROS_INFO("\nCurrent Type changed %dx%d", m_u32Width, m_u32Height);
            GUID guid;
            winrt::com_ptr<IMFTransform> spTransform;
            int i = 0;

            /*while (MF_E_INVALIDINDEX != spSourceReader.try_as<IMFSourceReaderEx>()->GetTransformForStream(MF_SOURCE_READER_FIRST_VIDEO_STREAM, i++, &guid, spTransform.put()))
            {
                ROS_INFO("\nTranform %d: %x-%x-%x-%x%x%x%x", i - 1, guid.Data1, guid.Data2, guid.Data3, guid.Data4[0], guid.Data4[1], guid.Data4[2], guid.Data4[3]);
            }
            ROS_INFO("\nNumber of tranforms in Chain: %d", i - 1);*/

            m_u32SourceReaderFlags = MF_SOURCE_READER_CONTROLF_DRAIN;

        }
        if (MF_SOURCE_READERF_ENDOFSTREAM & dwStreamFlags)
        {
            ////TODO: need to handle EOS
            //throw_hresult(MF_E_END_OF_STREAM);
            m_captureCallbackEvent(hresult_error(MF_E_END_OF_STREAM), L"End Of stream");
        }
    }
    catch (hresult_error const& ex)
    {
        ROS_ERROR("%x:%s", ex.code(), winrt::to_string(ex.message()).c_str());
        m_captureCallbackEvent(ex, L":Trying to read sample in callback");
    }

    if (SUCCEEDED(hrStatus) && pSample)
    {
        PublishSample(pSample);
        m_configEvent();
        for (auto token : m_configEventTokenList)
        {
            m_configEvent.remove(token);
        }
    }
    LeaveCriticalSection(&m_critsec);
    HRESULT hr = spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, m_u32SourceReaderFlags, NULL, NULL, NULL, NULL);
    if(FAILED(hr))
    {
        hresult_error ex(hr);
        ROS_ERROR("%x:%s", ex.code(), winrt::to_string(ex.message()).c_str());
        m_captureCallbackEvent(ex, L":Trying to read sample in callback");
    }
    return S_OK;
}
bool WindowsCapture::SetResolution(int32_t width, int32_t height, float frameRate)
{
    EnterCriticalSection(&m_critsec);
    // post a delegate to be processed by the sample processing thread, so the order of mediatype change is maintained
    m_configEventTokenList.Append(m_configEvent.add([this, width, height, frameRate]()
        {
            HRESULT hr = S_OK;
            winrt::com_ptr<IMFMediaType> spMediaType;
            UINT32 FRNum, FRDen;
            int idx = 0;
            ROS_INFO("\nSetting resolution :%dx%d@%d", width, height, (int)frameRate);
            while (S_OK == (hr = spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, idx, spMediaType.put())))
            {
                check_hresult(MFGetAttributeSize(spMediaType.get(), MF_MT_FRAME_SIZE, &m_u32Width, &m_u32Height));
                check_hresult(MFGetAttributeRatio(spMediaType.get(), MF_MT_FRAME_RATE, &FRNum, &FRDen));
                float rate = ((float)FRNum / (float)FRDen);
                ROS_INFO("\nGot supported resolution :%dx%d@%d", m_u32Width, m_u32Height, (int)rate);
                if ((m_u32Width == width) && (height == m_u32Height) && ((int)rate == (int)frameRate))
                {
                    ROS_INFO("Found matching resolution!");
                    break;
                }
                idx++;
            }
            if (FAILED(hr))
            {
                ROS_WARN("No matching resolution supported by source. Setting up a conversion");
                GUID subtype;
                winrt::com_ptr<IMFMediaType> spMediaType;
                m_u32Width = width;
                m_u32Height = height;
                check_hresult(spSourceReader->GetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, MF_SOURCE_READER_CURRENT_TYPE_INDEX, spMediaType.put()));
                check_hresult(spMediaType->GetGUID(MF_MT_SUBTYPE, &subtype));
                check_hresult(MFSetAttributeSize(spMediaType.get(), MF_MT_FRAME_SIZE, m_u32Width, m_u32Height));
                check_hresult(spMediaType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));
                check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMediaType.get()));
            }
            else
            {

                DWORD flags = 0;
                GUID subtype;
                HRESULT hr = spSourceReader.as<IMFSourceReaderEx>()->SetNativeMediaType(MF_SOURCE_READER_FIRST_VIDEO_STREAM, spMediaType.get(), &flags);

                if (SUCCEEDED(hr))
                {
                    ROS_INFO("Native res set");
                }
                else
                {
                    ROS_WARN("Couldn't set native res: %x", hr);
                }

                check_hresult(spMediaType->GetGUID(MF_MT_SUBTYPE, &subtype));
                if (!IsEqualGUID(subtype, MFVideoFormat_ARGB32))
                {
                    check_hresult(spMediaType->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));
                    check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMediaType.get()));
                }

            }
        }));
    LeaveCriticalSection(&m_critsec);
    return true;
}

} // namespace ros_win_camera
