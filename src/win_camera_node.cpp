#include "capture.h"

const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 4;

int main(int argc, char **argv)
{
      winrt::slim_mutex waitForFinish;
      boost::shared_ptr<ros::Rate> rate;
      double hz(DEFAULT_RATE);

      ros::init(argc, argv, "win_camera");
      ros::NodeHandle private_node("~");
      
      std::string device_path("");
      std::string frame_id("camera");
      std::string file_path("");
      private_node.getParam("frame_id", frame_id);
      private_node.getParam("rate", hz);

      int32_t image_width(640);
      int32_t image_height(480);

      winrt::com_ptr<ros_win_camera::WindowsCapture> camera;
      camera.attach(new ros_win_camera::WindowsCapture(private_node,
          "image_raw",
          PUBLISHER_BUFFER_SIZE,
          frame_id));
  
  try
  {
      waitForFinish.lock();

      auto handler = [&](winrt::hresult_error ex, winrt::hstring msg)
      {
          if (ex.code() == MF_E_END_OF_STREAM)
          {
              ROS_INFO("\nEOS");
          }
          else
          {
              ROS_ERROR("%s:%x:%s", winrt::to_string(msg).c_str(), ex.code(), winrt::to_string(ex.message()).c_str());
          }
          waitForFinish.unlock();
      };

      if (private_node.getParam("file", file_path) && file_path != "")
      {
          camera->OpenFile(file_path, handler);
      }
      else if (private_node.getParam("device_path", device_path) && device_path != "")
      {
          camera->Open(device_path, handler);
      }
      else
      {
          camera->Open("", handler);
      }

      rate.reset(new ros::Rate(hz));
      if (!((private_node.getParam("image_width", image_width))
          && (private_node.getParam("image_height", image_height)))
          )
      {
          image_width = 640;
          image_height = 480;
      }

      if (!camera->SetResolution(image_width, image_height, (1000000000.0f / rate->expectedCycleTime().nsec)))
      {
          ROS_WARN("fail to set res");
      }

      waitForFinish.lock();

  }
  catch (winrt::hresult_error const& e)
  {
      ROS_ERROR_STREAM("camera open failed: " << winrt::to_string(e.message()));
      return 1;
  }
  catch(std::exception const &e)
  {
    ROS_ERROR_STREAM("camera open failed: " << e.what());
    return 1;
  }

  return 0;
}
