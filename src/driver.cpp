
#include "driver.h"
#include <string>

namespace
{
const double DEFAULT_RATE = 30.0;
const int32_t PUBLISHER_BUFFER_SIZE = 4;
}

namespace ros_win_camera
{

Driver::Driver(ros::NodeHandle &private_node, ros::NodeHandle &camera_node)
    : private_node_(private_node),
      camera_node_(camera_node)
{
}

void Driver::setup(winrt::delegate<winrt::hresult_error, winrt::hstring> handler)
{
    try
    {
        double hz(DEFAULT_RATE);
        std::string device_path("");
        std::string frame_id("camera");
        std::string file_path("");

        private_node_.getParam("frame_id", frame_id);
        private_node_.getParam("rate", hz);

        int32_t image_width(640);
        int32_t image_height(480);

        camera_.attach(new WindowsCapture(camera_node_,
            "image_raw",
            PUBLISHER_BUFFER_SIZE,
            frame_id));

        if (private_node_.getParam("file", file_path) && file_path != "")
        {
            camera_->OpenFile(file_path,handler);
        }
        else if (private_node_.getParam("device_path", device_path) && device_path != "")
        {
            camera_->Open(device_path,handler);
        }
        else
        {
            camera_->Open("",handler);
        }

        rate_.reset(new ros::Rate(hz));

        

        //  camera_->setPropertyFromParam(CV_CAP_PROP_POS_MSEC, "cv_cap_prop_pos_msec");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_POS_AVI_RATIO, "cv_cap_prop_pos_avi_ratio");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_WIDTH, "cv_cap_prop_frame_width");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_HEIGHT, "cv_cap_prop_frame_height");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FPS, "cv_cap_prop_fps");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FOURCC, "cv_cap_prop_fourcc");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FRAME_COUNT, "cv_cap_prop_frame_count");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_FORMAT, "cv_cap_prop_format");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_MODE, "cv_cap_prop_mode");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_BRIGHTNESS, "cv_cap_prop_brightness");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_CONTRAST, "cv_cap_prop_contrast");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_SATURATION, "cv_cap_prop_saturation");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_HUE, "cv_cap_prop_hue");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_GAIN, "cv_cap_prop_gain");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_EXPOSURE, "cv_cap_prop_exposure");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_CONVERT_RGB, "cv_cap_prop_convert_rgb");
        //
        //  camera_->setPropertyFromParam(CV_CAP_PROP_RECTIFICATION, "cv_cap_prop_rectification");
        //  camera_->setPropertyFromParam(CV_CAP_PROP_ISO_SPEED, "cv_cap_prop_iso_speed");
        //#ifdef CV_CAP_PROP_WHITE_BALANCE_U
        //  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_U, "cv_cap_prop_white_balance_u");
        //#endif // CV_CAP_PROP_WHITE_BALANCE_U
        //#ifdef CV_CAP_PROP_WHITE_BALANCE_V
        //  camera_->setPropertyFromParam(CV_CAP_PROP_WHITE_BALANCE_V, "cv_cap_prop_white_balance_v");
        //#endif // CV_CAP_PROP_WHITE_BALANCE_V
        //#ifdef CV_CAP_PROP_BUFFERSIZE
        //  camera_->setPropertyFromParam(CV_CAP_PROP_BUFFERSIZE, "cv_cap_prop_buffersize");
        //#endif // CV_CAP_PROP_BUFFERSIZE
    }
    catch (winrt::hresult_error const& ex)
    {
        if (ex.code() == S_OK)
        {
            handler(ex, L"End-Of-Stream");
        }
        handler(ex, L"Error");
    }
}

//void Driver::proceed()
//{
//  if (camera_->capture())
//  {
//    camera_->publish();
//  }
//  //rate_->sleep();
//}

Driver::~Driver()
{
}

} // namespace cv_camera
