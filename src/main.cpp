// Copyright (C) Microsoft Corporation. All rights reserved.
#include <win_camera/win_camera_node.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<win_camera::CameraDriver>(
            rclcpp::NodeOptions()));
    rclcpp::shutdown();

    return 0;
}
