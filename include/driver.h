// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#ifndef CV_CAMERA_DRIVER_H
#define CV_CAMERA_DRIVER_H

#include "capture.h"

namespace ros_win_camera
{

/**
 * @brief ROS cv camera driver.
 *
 * This wraps getting parameters and publish in specified rate.
 */
class Driver
{
 public:
  /**
   * @brief construct with ROS node handles.
   *
   * use private_node for getting topics like ~rate or ~device,
   * camera_node for advertise and publishing images.
   *
   * @param private_node node for getting parameters.
   * @param camera_node node for publishing.
   */
  Driver(ros::NodeHandle& private_node,
         ros::NodeHandle& camera_node);
  ~Driver();

  /**
   * @brief Setup camera device and ROS parameters.
   *
   * @throw cv_camera::DeviceError device open failed.
   */
  void setup(winrt::delegate<winrt::hresult_error, winrt::hstring>);
  /**
   * @brief Capture, publish and sleep
   */
  //void proceed();
 private:
  /**
   * @brief ROS private node for getting ROS parameters.
   */
  ros::NodeHandle private_node_;
  /**
   * @brief ROS private node for publishing images.
   */
  ros::NodeHandle camera_node_;
  /**
   * @brief wrapper of cv::VideoCapture.
   */
  winrt::com_ptr<WindowsCapture> camera_;

  /**
   * @brief publishing rate.
   */
  boost::shared_ptr<ros::Rate> rate_;
};

}  // namespace cv_camera

#endif  // CV_CAMERA_DRIVER_H
