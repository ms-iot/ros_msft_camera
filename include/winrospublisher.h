// Copyright (C) Microsoft Corporation. All rights reserved.
#include <mfapi.h>
#include <winrt\base.h>

#include <image_transport/image_transport.h>
#include <compressed_image_transport/compressed_publisher.h>
#include <compressed_image_transport/compression_common.h>
#include <compressed_image_transport/CompressedPublisherConfig.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <std_msgs/UInt64.h>
#include <memory>
#include <queue>
using namespace winrt;
namespace enc = sensor_msgs::image_encodings;
namespace ros_msft_camera
{
    class WinRosPublisherBase
    {
    public:
        WinRosPublisherBase(ros::NodeHandle& node, const std::string& topic_name, int32_t buffer_size, const std::string& frame_id, std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager)
            : m_nodeHandle(node),
            m_topicName(topic_name),
            m_i32BufferSize(buffer_size),
            m_frameID(frame_id),
            m_spCameraInfoManager(pCameraInfoManager)
        {
        }
        virtual ~WinRosPublisherBase() = default;
        virtual void OnSample(IMFSample *pSample, UINT32 u32Width, UINT32 u32Height) = 0;

    protected:
        ros::NodeHandle m_nodeHandle;
        std::string m_topicName;
        std::string m_frameID;
        int32_t m_i32BufferSize;

        sensor_msgs::CameraInfo m_cameraInfo;
        std::shared_ptr<camera_info_manager::CameraInfoManager> m_spCameraInfoManager;
        
    };

    class WinRosPublisherImageRaw : WinRosPublisherBase
    {
    public:
        WinRosPublisherImageRaw(ros::NodeHandle& node, const std::string& topic_name, int32_t buffer_size, const std::string& frame_id, std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager)
            : WinRosPublisherBase(node,topic_name,buffer_size,frame_id, pCameraInfoManager)
            , m_imageTransport(m_nodeHandle)
        {
            m_bRescaleCameraInfo = m_nodeHandle.param<bool>("rescale_camera_info", false);
            m_cameraPublisher = m_imageTransport.advertiseCamera(m_topicName, m_i32BufferSize);
        }
        virtual ~WinRosPublisherImageRaw() = default;
        void OnSample(IMFSample* pSample, UINT32 u32Width, UINT32 u32Height)
        {
            try 
            {
                if (pSample)
                {
                    winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
                    winrt::com_ptr<IMF2DBuffer2> spMediaBuf2d;
                    uint32_t littleEndian = 1;
                    BYTE* pix;
                    LONG Stride;
                    ros::Time now = ros::Time::now();

                    m_cameraInfo = m_spCameraInfoManager->getCameraInfo();
                    if (m_cameraInfo.height == 0 && m_cameraInfo.width == 0)
                    {
                        m_cameraInfo.height = u32Height;
                        m_cameraInfo.width = u32Width;
                    }
                    else if (m_cameraInfo.height != u32Height || m_cameraInfo.width != u32Width)
                    {
                        if (m_bRescaleCameraInfo)
                        {
                            int old_width = m_cameraInfo.width;
                            int old_height = m_cameraInfo.height;
                            RescaleCameraInfo(u32Width, u32Height);
                            ROS_INFO_ONCE("Camera calibration automatically rescaled from %dx%d to %dx%d",
                                old_width, old_height, u32Width, u32Height);
                        }
                        else
                        {
                            ROS_WARN_ONCE("Calibration resolution %dx%d does not match camera resolution %dx%d. "
                                "Use rescale_camera_info param for rescaling",
                                m_cameraInfo.width, m_cameraInfo.height, u32Width, u32Height);
                        }
                    }
                    m_cameraInfo.header.stamp = now;
                    m_cameraInfo.header.frame_id = m_frameID;

                    check_hresult(pSample->GetBufferByIndex(0, spMediaBuf.put()));
                    spMediaBuf2d = spMediaBuf.as<IMF2DBuffer2>();

                    sensor_msgs::ImagePtr ros_image = boost::make_shared<sensor_msgs::Image>();

                    ros_image->header = m_cameraInfo.header;
                    ros_image->height = u32Height;
                    ros_image->width = u32Width;
                    ros_image->encoding = enc::BGRA8;
                    ros_image->is_bigendian = !*((uint8_t*)&littleEndian);

                    check_hresult(spMediaBuf2d->Lock2D(&pix, &Stride));
                    if (Stride < 0)
                    {
                        ros_image->step = -Stride;
                        size_t size = ros_image->step * u32Height;
                        ros_image->data.resize(size);
                        for (uint32_t i = 0; i < u32Height; i++)
                        {
                            memcpy(ros_image->data.data() + i * ros_image->step, pix + Stride * i, -Stride);
                        }
                    }
                    else
                    {
                        ros_image->step = Stride;
                        size_t size = Stride * u32Height;
                        ros_image->data.resize(size);
                        memcpy(ros_image->data.data(), pix, size);

                    }
                    check_hresult(spMediaBuf2d->Unlock2D());
                    m_cameraPublisher.publish(*ros_image, m_cameraInfo);
                }
            }
            catch (winrt::hresult_error const& ex)
            {
                ROS_ERROR("Error Publishing Sample: %x:%s",ex.code().value, winrt::to_string( ex.message().c_str()));
            }
        }

    private:
        void RescaleCameraInfo(int width, int height)
        {
            double widthCoeff = static_cast<double>(width) / m_cameraInfo.width;
            double heightCoeff = static_cast<double>(height) / m_cameraInfo.height;
            m_cameraInfo.width = width;
            m_cameraInfo.height = height;

            // ref: http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
            m_cameraInfo.K[0] *= widthCoeff;
            m_cameraInfo.K[2] *= widthCoeff;
            m_cameraInfo.K[4] *= heightCoeff;
            m_cameraInfo.K[5] *= heightCoeff;

            m_cameraInfo.P[0] *= widthCoeff;
            m_cameraInfo.P[2] *= widthCoeff;
            m_cameraInfo.P[5] *= heightCoeff;
            m_cameraInfo.P[6] *= heightCoeff;
        }

        image_transport::ImageTransport m_imageTransport;
        image_transport::CameraPublisher m_cameraPublisher;
        bool m_bRescaleCameraInfo;
    };

    class WinRosPublisherMFSample : WinRosPublisherBase
    {
    public:
        WinRosPublisherMFSample(ros::NodeHandle& node, const std::string& topic_name, int32_t queue_size, const std::string& frame_id, std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager)
            : WinRosPublisherBase(node, topic_name, queue_size, frame_id, pCameraInfoManager)
            , m_queueSize(queue_size)
        {
            m_MFSamplePublisher = m_nodeHandle.advertise<std_msgs::UInt64>(topic_name, queue_size);
        }
        virtual ~WinRosPublisherMFSample() = default;
        void OnSample(IMFSample* pSample, UINT32 u32Width, UINT32 u32Height)
        {
            m_sampleQueue.emplace(nullptr);
            m_sampleQueue.back().copy_from(pSample);
            std_msgs::UInt64 sampleMsg;
            sampleMsg.data = (UINT64)pSample;
            m_MFSamplePublisher.publish(sampleMsg);
            if (m_sampleQueue.size() > m_queueSize)
            {
                m_sampleQueue.pop();
            }
        }
    private:
        ros::Publisher m_MFSamplePublisher;
        std::queue<winrt::com_ptr<IMFSample>> m_sampleQueue;
        size_t m_queueSize;
    };

    class WinRosPublisherCompressed : WinRosPublisherBase
    {
    public:
        WinRosPublisherCompressed(ros::NodeHandle& node, const std::string& topic_name, int32_t buffer_size, const std::string& frame_id, std::shared_ptr<camera_info_manager::CameraInfoManager> pCameraInfoManager)
            : WinRosPublisherBase(node, topic_name, buffer_size, frame_id, pCameraInfoManager)
            , m_imageTransport(m_nodeHandle)
        {
            m_compressedPublisher = m_nodeHandle.advertise<sensor_msgs::CompressedImage>("compressed/jpeg", buffer_size);
        }
        virtual ~WinRosPublisherCompressed() = default;

        void OnSample(IMFSample* pSample, UINT32 u32Width, UINT32 u32Height)
        {
            winrt::com_ptr<IMFMediaBuffer> spMediaBuf;
            sensor_msgs::CompressedImagePtr ros_compressedImage = boost::make_shared<sensor_msgs::CompressedImage>();
            ros_compressedImage->header = m_cameraInfo.header;
            ros_compressedImage->format = compressed_image_transport::CompressedPublisher_jpeg;
            check_hresult(pSample->GetBufferByIndex(0, spMediaBuf.put()));
            BYTE* pData;
            DWORD dwMaxLen, dwCurrLen;
            spMediaBuf->Lock(&pData, &dwMaxLen, &dwCurrLen);
            ros_compressedImage->data.resize(dwCurrLen);
            memcpy(ros_compressedImage->data.data(), pData, dwCurrLen);
            spMediaBuf->Unlock();
            m_compressedPublisher.publish(ros_compressedImage);
            
        }
    private:
        ros::Publisher m_compressedPublisher;
        image_transport::ImageTransport m_imageTransport;
    };

}