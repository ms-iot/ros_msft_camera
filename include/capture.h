
#ifndef WIN_CAMERA_CAPTURE_H
#define WIN_CAMERA_CAPTURE_H
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
#include <string>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <win_camera\MFSample.h>
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
    class WindowsCapture : private IMFSourceReaderCallback
    {
    public:

        /**
         * @brief costruct with ros node and topic settings
         *
         * @param node ROS node handle for advertise topic.
         * @param topic_name name of topic to publish (this may be image_raw).
         * @param buffer_size size of publisher buffer.
         * @param frame_id frame_id of publishing messages.
         */
        WindowsCapture(ros::NodeHandle& node,
            const std::string& topic_name,
            int32_t buffer_size,
            const std::string& frame_id);

         /**
          * @brief Open capture device with device name.
          *
          * @param device_path path of the camera device
          */
        void Open(const std::string& device_path, winrt::delegate<winrt::hresult_error, winrt::hstring> handler);

        /**
         * @brief Load camera info from file.
         *
         * This loads the camera info from the file specified in the camera_info_url parameter.
         */
        void loadCameraInfo();

         /**
          * @brief open video file instead of capture device.
          */
        void OpenFile(const std::string& file_path, winrt::delegate<winrt::hresult_error, winrt::hstring> handler);

        /**
         * @brief capture an image and store.
         *
         * to publish the captured image, call publish();
         * @return true if success to capture, false if not captured.
         */
        bool PublishSample(IMFSample *pSample);

        /**
         * @brief accessor of CameraInfo.
         *
         * you have to call capture() before call this.
         *
         * @return CameraInfo
         */
        inline const sensor_msgs::CameraInfo& GetCameraInfo() const
        {
            return m_cameraInfo;
        }

        /**
         * @brief try set capture image resolution
         * @return true if success
         */
        bool SetResolution(int32_t width, int32_t height, float frameRate);

        // IUnknown methods
        STDMETHODIMP QueryInterface(REFIID iid, void** ppv)
        {
            static const QITAB qit[] =
            {
                QITABENT(WindowsCapture, IMFSourceReaderCallback),
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

    private:
        // private destructor so that destruction is controlled by Release() as we inherit from IUnknown
        virtual ~WindowsCapture()
        {

        }

        void WindowsCapture::StartStreaming();
        /**
         * @brief rescale camera calibration to another resolution
         */
        void RescaleCameraInfo(int width, int height);

        /**
         * @brief node handle for advertise.
         */
        ros::NodeHandle m_nodeHandle;

        /**
         * @brief ROS image transport utility.
         */
        image_transport::ImageTransport m_imageTransport;

        /**
         * @brief name of topic without namespace (usually "image_raw").
         */
        std::string m_topicName;

        /**
         * @brief header.frame_id for publishing images.
         */
        std::string m_frameID;
        /**
         * @brief size of publisher buffer
         */
        int32_t m_i32BufferSize;

        /**
         * @brief image publisher created by image_transport::ImageTransport.
         */
        image_transport::CameraPublisher m_cameraPublisher;
        ros::Publisher m_MFSamplePublisher;
        /**
       * @brief this stores last captured image info.
       *
       * currently this has image size (width/height) only.
       */
        sensor_msgs::CameraInfo m_cameraInfo;

        /**
         * @brief camera info manager
         */
        camera_info_manager::CameraInfoManager m_cameraInfoManager;

        /**
         * @brief rescale_camera_info param value
         */
        bool m_bRescaleCameraInfo;
        winrt::com_ptr<IMFMediaSource> spMediaSource;
        winrt::com_ptr <IMFSourceReader> spSourceReader;
        //winrt::com_ptr<IMFSample> spSample;
        UINT32 m_u32Width, m_u32Height;
        UINT32 m_u32SourceReaderFlags;

        winrt::event<winrt::delegate<>> m_configEvent;
        winrt::Windows::Foundation::Collections::IVector<winrt::event_token> m_configEventTokenList;

        winrt::event<winrt::delegate<winrt::hresult_error, winrt::hstring>> m_captureCallbackEvent;


        // IMFSourceReaderCallback methods

        STDMETHODIMP OnEvent(DWORD dwStreamIndex, IMFMediaEvent* mediaEvt)
        {
            MediaEventType evt;
            mediaEvt->GetType(&evt);
            ROS_INFO("Event Recvd: %d", evt);
            return S_OK;
        }

        STDMETHODIMP OnFlush(DWORD)
        {
            ROS_INFO("Flush Received");
            return S_OK;
        }

        STDMETHODIMP OnReadSample(HRESULT hrStatus, DWORD dwStreamIndex,
            DWORD dwStreamFlags, LONGLONG llTimestamp, IMFSample* pSample);

        long                m_nRefCount;
        CRITICAL_SECTION    m_critsec;
    };

} // namespace ros_win_camera

#endif // WIN_CAMERA_CAPTURE_H
