// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>
#include "cv_camera/capture.h"
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

Capture::Capture(ros::NodeHandle &node, const std::string &topic_name,
                 int32_t buffer_size, const std::string &frame_id)
    : node_(node),
      it_(node_),
      topic_name_(topic_name),
      buffer_size_(buffer_size),
      frame_id_(frame_id),
      info_manager_(node_, frame_id),
      m_nRefCount(1)
{
    InitializeCriticalSection(&m_critsec);
}

void Capture::loadCameraInfo()
{
  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }

  rescale_camera_info_ = node_.param<bool>("rescale_camera_info", false);

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

void Capture::rescaleCameraInfo(int width, int height)
{
  double width_coeff = static_cast<double>(width) / info_.width;
  double height_coeff = static_cast<double>(height) / info_.height;
  info_.width = width;
  info_.height = height;

  // See http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html for clarification
  info_.K[0] *= width_coeff;
  info_.K[2] *= width_coeff;
  info_.K[4] *= height_coeff;
  info_.K[5] *= height_coeff;

  info_.P[0] *= width_coeff;
  info_.P[2] *= width_coeff;
  info_.P[5] *= height_coeff;
  info_.P[6] *= height_coeff;
}

void Capture::open(const std::string& device_path)
{
    ROS_INFO("\nin cap open");

    winrt::com_ptr<IMFAttributes> spAttributes;
    winrt::com_ptr<IMFAttributes> spSRAttributes;

    BOOL fStreamStarted = FALSE;
    BOOL fSeletect = FALSE;

    winrt::hstring cameraSymbolicLink;
    auto filteredDevices = DeviceInformation::FindAllAsync(DeviceClass::VideoCapture).get();
    if (!filteredDevices.Size())
    {
        throw_hresult(MF_E_NO_CAPTURE_DEVICES_AVAILABLE);
    }
    auto deviceIter = filteredDevices.First();
    if ((!device_path.empty()) && (device_path.c_str() != nullptr))
    {
        ROS_INFO("\nLooking for: %s", device_path.c_str());
        while (deviceIter.HasCurrent())
        {
            auto device = deviceIter.Current();
            if (winrt::to_string(device.Id()).compare(device_path) == 0)
            {
                cameraSymbolicLink = device.Id();
            }
            deviceIter.MoveNext();
        }
    }
    else
    {
        cameraSymbolicLink = deviceIter.Current().Id();
    }

    ROS_INFO("\nOpening: %s", winrt::to_string(cameraSymbolicLink).c_str());
    // Create attribute store
    check_hresult(MFCreateAttributes(spAttributes.put(), 16));
    // Set required guids for video camera
    check_hresult(spAttributes->SetGUID(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE, MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_GUID));
    check_hresult(spAttributes->SetString(MF_DEVSOURCE_ATTRIBUTE_SOURCE_TYPE_VIDCAP_SYMBOLIC_LINK, cameraSymbolicLink.c_str()));

    MFCreateDeviceSource(spAttributes.get(), spMediaSource.put());
    check_hresult(MFCreateAttributes(spSRAttributes.put(), 3));
    spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE);
    spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE);
    spSRAttributes->SetUnknown(MF_SOURCE_READER_ASYNC_CALLBACK, this);
    check_hresult(MFCreateSourceReaderFromMediaSource(spMediaSource.get(), spSRAttributes.get(), spSourceReader.put()));

    check_hresult(spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, spMT.put()));
    GUID subtype;
    spMT->GetGUID(MF_MT_SUBTYPE, &subtype);
    check_hresult(spMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));

    try
    {
        check_hresult(spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMT.get()));
        check_hresult(spSourceReader->SetStreamSelection((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, TRUE));
    }
    catch (hresult_error const& ex)
    {
        std::wcout << L"\nError: " << std::hex << ex.code() << L":" << ex.message().c_str();
    }



    pub_ = it_.advertiseCamera(topic_name_, buffer_size_);
    loadCameraInfo();
    ROS_INFO("\nCamera open Done");

    spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, NULL, NULL, NULL, NULL);
}


void Capture::openFile(const std::string &file_path, winrt::delegate<> frameArrived/* = nullptr*/)
{
    winrt::com_ptr<IMFAttributes> spSRAttributes;
    GUID subtype;
    check_hresult(MFCreateAttributes(spSRAttributes.put(), 2));
    spSRAttributes->SetUINT32(MF_SOURCE_READER_ENABLE_ADVANCED_VIDEO_PROCESSING, TRUE);
    spSRAttributes->SetUINT32(MF_LOW_LATENCY, TRUE);
    MFCreateSourceReaderFromURL(winrt::to_hstring(file_path).c_str(),spSRAttributes.get(),spSourceReader.put());
    check_hresult(spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, spMT.put()));
    spMT->GetGUID(MF_MT_SUBTYPE, &subtype);
    check_hresult(spMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32));

    pub_ = it_.advertiseCamera(topic_name_, buffer_size_);

  std::string url;
  if (node_.getParam("camera_info_url", url))
  {
    if (info_manager_.validateURL(url))
    {
      info_manager_.loadCameraInfo(url);
    }
  }
}

bool Capture::capture()
{
    if (spSample)
    {
        UINT32 Width, Height;
        MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &Width, &Height);

        ros::Time now = ros::Time::now();

        info_ = info_manager_.getCameraInfo();

        if (info_.height == 0 && info_.width == 0)
        {
            info_.height = Height;
            info_.width = Width;
        }
        else if (info_.height != Height || info_.width != Width)
        {
            if (rescale_camera_info_)
            {
                int old_width = info_.width;
                int old_height = info_.height;
                rescaleCameraInfo(Width, Height);
                ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                    old_width, old_height, Width, Height);
            }
            else
            {
                ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                    "Use rescale_camera_info param for rescaling",
                    info_.width, info_.height, Width, Height);
            }
        }
        info_.header.stamp = now;
        info_.header.frame_id = frame_id_;
        return true;
    }
    return false;
}

void Capture::publish()
{
    if (spSample)
    {
        winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
        winrt::com_ptr<IMF2DBuffer2> spMediaBuf2d;
        UINT32 Width, Height;
        uint32_t littleEndian = 1;
        MFGetAttributeSize(spMT.get(), MF_MT_FRAME_SIZE, &Width, &Height);

        BYTE* pix;
        LONG Stride;
        spSample->GetBufferByIndex(0, spMediaBuf.put());
        spMediaBuf2d = spMediaBuf.as<IMF2DBuffer2>();
        spMediaBuf2d->Lock2D(&pix, &Stride);
        
        sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();
        ros_image->header = info_.header;
        ros_image->height = Height;
        ros_image->width = Width;
        ros_image->encoding = enc::BGRA8;
        ros_image->is_bigendian = !*((uint8_t *)&littleEndian);// (boost::endian::order::native == boost::endian::order::big);
        ros_image->step = Stride;
        size_t size = Stride * Height;
        ros_image->data.resize(size);

        //if (image.isContinuous())
        {
            memcpy((char*)(&ros_image->data[0]), pix, size);
        }
        spMediaBuf2d->Unlock2D();
        pub_.publish(*ros_image, info_);
    }
}

HRESULT Capture::OnReadSample(
    HRESULT hrStatus,
    DWORD /* dwStreamIndex */,
    DWORD dwStreamFlags,
    LONGLONG llTimestamp,
    IMFSample* pSample      // Can be NULL
)
{
    EnterCriticalSection(&m_critsec);

    if (SUCCEEDED(hrStatus))
    {
        if (pSample)
        {
            spSample.copy_from(pSample);
        }
    }
    else
    {
        throw_hresult(hrStatus);
    }

    if (MF_SOURCE_READERF_ENDOFSTREAM & dwStreamFlags)
    {
        throw_hresult(MF_E_END_OF_STREAM);
    }


    LeaveCriticalSection(&m_critsec);
    capture();
    publish();
    spSourceReader->ReadSample(MF_SOURCE_READER_FIRST_VIDEO_STREAM, 0, NULL, NULL, NULL, NULL);
    return S_OK;
}

//bool Capture::setPropertyFromParam(int property_id, const std::string &param_name)
//{
//  //if (cap_.isOpened())
//  //{
//  //  double value = 0.0;
//  //  if (node_.getParam(param_name, value))
//  //  {
//  //    ROS_INFO("setting property %s = %lf", param_name.c_str(), value);
//  //    return cap_.set(property_id, value);
//  //  }
//  //}
//  return true;
//}

} // namespace cv_camera
