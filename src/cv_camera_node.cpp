// Copyright [2015] Takashi Ogura<t.ogura@gmail.com>

#include "cv_camera/driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cv_camera");
  ros::NodeHandle private_node("~");
  ros_win_camera::Driver driver(private_node, private_node);
  winrt::slim_mutex waitForDriver;
  try
  {
      waitForDriver.lock();
      driver.setup([&](winrt::hresult_error ex, winrt::hstring msg) 
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

      waitForDriver.lock();

  }
  catch (winrt::hresult_error const& e)
  {
      ROS_ERROR_STREAM("cv camera open failed: " << winrt::to_string(e.message()));
      return 1;
  }
  catch(std::exception const &e)
  {
    ROS_ERROR_STREAM("cv camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
