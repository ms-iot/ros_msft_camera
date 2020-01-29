// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_CAPTURE_H
#define CV_CAMERA_CAPTURE_H
#include <mfapi.h>
#include <mfidl.h>
#include <mfreadwrite.h>
#include <mferror.h>
#include <winrt\base.h>
#include <winrt\Windows.Foundation.Collections.h>
#include <winrt\Windows.Devices.Enumeration.h>
#include <winrt\Windows.Media.Capture.h>
#include <winrt\Windows.Media.Capture.Frames.h>
#include <winrt\Windows.Graphics.Imaging.h>

//#include "cv_camera/exception.h"
#include <string>

#include <ros/ros.h>
//#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
//#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <Shlwapi.h>
/**
 * @brief namespace of this package
 */
namespace ros_win_camera
{

/**
 * @brief captures by cv::VideoCapture and publishes to ROS topic.
 *
 */
class Capture : public IMFSourceReaderCallback
{
public:
    // IUnknown methods
    STDMETHODIMP QueryInterface(REFIID iid, void** ppv)
    {
        static const QITAB qit[] =
        {
            QITABENT(Capture, IMFSourceReaderCallback),
            { 0 },
        };
        return QISearch(this, qit, iid, ppv);
    }
    STDMETHODIMP_(ULONG) AddRef()
    {
        return InterlockedIncrement(&m_nRefCount);
    }
    STDMETHODIMP_(ULONG) Release()
    {
        ULONG uCount = InterlockedDecrement(&m_nRefCount);
        if (uCount == 0)
        {
            delete this;
        }
        return uCount;
    }
    STDMETHODIMP OnEvent(DWORD, IMFMediaEvent*)
    {
        return S_OK;
    }

    STDMETHODIMP OnFlush(DWORD)
    {
        return S_OK;
    }
    // IMFSourceReaderCallback methods
    STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD dwStreamIndex,
        DWORD dwStreamFlags, LONGLONG llTimestamp, IMFSample* pSample);

    long                m_nRefCount;
    CRITICAL_SECTION    m_critsec;
  /**
   * @brief costruct with ros node and topic settings
   *
   * @param node ROS node handle for advertise topic.
   * @param topic_name name of topic to publish (this may be image_raw).
   * @param buffer_size size of publisher buffer.
   * @param frame_id frame_id of publishing messages.
   */
  Capture(ros::NodeHandle &node,
          const std::string &topic_name,
          int32_t buffer_size,
          const std::string &frame_id);

  /**
   * @brief Open capture device with device ID.
   *
   * @param device_id id of camera device (number from 0)
   * @throw cv_camera::DeviceError device open failed
   *
   */
  //void open(int32_t device_id, winrt::delegate<> frameArrived = nullptr);

  /**
   * @brief Open capture device with device name.
   *
   * @param device_path path of the camera device
   * @throw cv_camera::DeviceError device open failed
   */
  void open(const std::string &device_path);

  /**
   * @brief Load camera info from file.
   *
   * This loads the camera info from the file specified in the camera_info_url parameter.
   */
  void loadCameraInfo();

  /**
   * @brief Open default camera device.
   *
   * This opens with device 0.
   *
   * @throw cv_camera::DeviceError device open failed
   */
  //void open();

  /**
   * @brief open video file instead of capture device.
   */
  void openFile(const std::string &file_path, winrt::delegate<> frameArrived = nullptr);

  /**
   * @brief capture an image and store.
   *
   * to publish the captured image, call publish();
   * @return true if success to capture, false if not captured.
   */
  bool capture();

  /**
   * @brief Publish the image that is already captured by capture().
   *
   */
  void publish();

  /**
   * @brief accessor of CameraInfo.
   *
   * you have to call capture() before call this.
   *
   * @return CameraInfo
   */
  inline const sensor_msgs::CameraInfo &getInfo() const
  {
    return info_;
  }

  /**
   * @brief accessor of cv::Mat
   *
   * you have to call capture() before call this.
   *
   * @return captured cv::Mat
   */
  /*inline const cv::Mat &getCvImage() const
  {
    return bridge_.image;
  }*/

  /**
   * @brief accessor of ROS Image message.
   *
   * you have to call capture() before call this.
   *
   * @return message pointer.
   */
  /*inline const sensor_msgs::ImagePtr getImageMsgPtr() const
  {
    return bridge_.toImageMsg();
  }*/

  /**
   * @brief try capture image width
   * @return true if success
   */
  inline bool setResolution(int32_t width, int32_t height,float frameRate)
  {
      HRESULT hr = S_OK;
      winrt::com_ptr<IMFMediaType> spMediaType;
      UINT32 FRNum, FRDen;
      int idx = 0;
      ROS_INFO("\nSetting resolition :%dx%d@%d" ,width, height, (int)frameRate);
      while (S_OK == (hr = spSourceReader->GetNativeMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, idx, spMediaType.put())))
      {
          UINT32 Width, Height;
          MFGetAttributeSize(spMediaType.get(), MF_MT_FRAME_SIZE, &Width, &Height);
          MFGetAttributeRatio(spMediaType.get(), MF_MT_FRAME_RATE, &FRNum, &FRDen);
          float rate = ((float)FRNum / (float)FRDen);
          ROS_INFO("\nGot supported resolution :%dx%d@%d", Width, Height, (int)rate);
          if ((Width == width) && (height == Height) && ((int)rate == (int)frameRate))
          {
              ROS_INFO("Found matching resolution!");
              break;
          }
          idx++;
      }
      if (hr == S_OK)
      {
          spMT = spMediaType;
          spMT->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_ARGB32);
          spSourceReader->SetCurrentMediaType((DWORD)MF_SOURCE_READER_FIRST_VIDEO_STREAM, nullptr, spMT.get());
          return true;
      }
      else
      {
          ROS_ERROR("No matching resolution supported by camera");
          return false;
      }
  }

  /**
   * @brief set CV_PROP_*
   * @return true if success
   */
  //bool setPropertyFromParam(int property_id, const std::string &param_name);

private:
    virtual ~Capture()
    {

    }
  /**
   * @brief rescale camera calibration to another resolution
   */
  void rescaleCameraInfo(int width, int height);

  /**
   * @brief node handle for advertise.
   */
  ros::NodeHandle node_;

  /**
   * @brief ROS image transport utility.
   */
  image_transport::ImageTransport it_;

  /**
   * @brief name of topic without namespace (usually "image_raw").
   */
  std::string topic_name_;

  /**
   * @brief header.frame_id for publishing images.
   */
  std::string frame_id_;
  /**
   * @brief size of publisher buffer
   */
  int32_t buffer_size_;

  /**
   * @brief image publisher created by image_transport::ImageTransport.
   */
  image_transport::CameraPublisher pub_;

    /**
   * @brief this stores last captured image info.
   *
   * currently this has image size (width/height) only.
   */
  sensor_msgs::CameraInfo info_;

  /**
   * @brief camera info manager
   */
  camera_info_manager::CameraInfoManager info_manager_;

  /**
   * @brief rescale_camera_info param value
   */
  bool rescale_camera_info_;
  winrt::com_ptr<IMFMediaSource> spMediaSource;
  //winrt::event<winrt::delegate<>> frameArrivedEvent;
  //winrt::Windows::Media::Capture::MediaCapture mc;
  
  //winrt::Windows::Media::Capture::Frames::MediaFrameReader fr;
  //winrt::Windows::Graphics::Imaging::SoftwareBitmap sb;
  winrt::com_ptr <IMFSourceReader> spSourceReader;
  winrt::com_ptr<IMFSample> spSample;
  winrt::com_ptr <IMFMediaType> spMT;
};

} // namespace cv_camera

#endif // CV_CAMERA_CAPTURE_H
