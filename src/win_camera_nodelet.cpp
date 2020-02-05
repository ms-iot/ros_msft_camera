// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "capture.h"

#include <nodelet/nodelet.h>

namespace ros_win_camera
{

/**
 * @brief Nodelet version of cv_camera.
 */
    const int32_t PUBLISHER_BUFFER_SIZE = 4;
class CvCameraNodelet : public nodelet::Nodelet
{

public:
  CvCameraNodelet() 
  {
  }
  ~CvCameraNodelet()
  {
      waitForDriver.lock();
  }

private:
  /**
   * @brief Start capture/publish thread.
   */
  virtual void onInit()
  {
    waitForDriver.lock();
    std::string frame_id("camera");
    camera.attach(new WindowsCapture(getPrivateNodeHandle(),
        "image_raw", PUBLISHER_BUFFER_SIZE, frame_id));
    try
    {
        // TODO: nodelet also needs to access the node params for device path and file/url if required
        camera->Open("",[&](winrt::hresult_error ex, winrt::hstring msg)
            {
                if (ex.code() == MF_E_END_OF_STREAM)
                {
                    ROS_INFO("\nEOS");
                }
                else
                {
                    ROS_ERROR("%s:%d:%s", winrt::to_string(msg).c_str(), ex.code(), winrt::to_string(ex.message()).c_str());
                }
                waitForDriver.unlock();
            });

    }
    catch (winrt::hresult_error const &e)
    {
      NODELET_ERROR_STREAM("failed to open device... do nothing: " << winrt::to_string(e.message().c_str()));
    }
    catch (std::exception & ex)
    {
        std::cout << ex.what();
    }
  }


  winrt::slim_mutex waitForDriver;
  /**
   * @brief ROS cv camera driver.
   */
  winrt::com_ptr<WindowsCapture> camera;
  
};

} // end namespace cv_camera

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(ros_win_camera::CvCameraNodelet, nodelet::Nodelet)
