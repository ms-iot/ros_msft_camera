// Copyright (c) Microsoft Corporation.  All rights reserved.

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <gtest/gtest.h>
#include<thread>

TEST(MsftCameraNode, getImage)
{
    ros::NodeHandle node;
    std::mutex waitMutex;
    static std::condition_variable waitForCB;
    void (*cb)(const sensor_msgs::Image::ConstPtr & image) = [](const sensor_msgs::Image::ConstPtr& image)
    {
        EXPECT_EQ("MsftCamera2", image->header.frame_id);
        EXPECT_EQ(720, image->height);
        EXPECT_EQ(1280, image->width);
        EXPECT_EQ("bgra8", image->encoding);
        waitForCB.notify_all();
    };
    ros::Subscriber sub = node.subscribe("/msft_camera_no_yaml/image_raw",
        1,
        cb);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    std::unique_lock<std::mutex> ul(waitMutex);

    waitForCB.wait(ul);
    spinner.stop();

}

TEST(MsftCameraNodeTest, getCameraInfo)
{
    ros::NodeHandle node;
    std::mutex waitMutex;
    static std::condition_variable waitForCB;

    void (*cb)(const sensor_msgs::CameraInfo::ConstPtr & info)
        = [](const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        EXPECT_EQ("MsftCamera2", info->header.frame_id);
        // K
        EXPECT_EQ(9, info->K.size());
        EXPECT_NEAR(0.0, info->K.at(0), 0.001);
        EXPECT_NEAR(0.0, info->K.at(1), 0.001);
        EXPECT_NEAR(0.0, info->K.at(2), 0.001);
        // D
        EXPECT_EQ(0, info->D.size());
        EXPECT_EQ("", info->distortion_model);

        // width
        EXPECT_EQ(1280, info->width);
        EXPECT_EQ(720, info->height);
        waitForCB.notify_all();
    };

    ros::Subscriber sub = node.subscribe("/msft_camera_no_yaml/camera_info",
        1,
        cb);

    std::unique_lock<std::mutex> ul(waitMutex);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    waitForCB.wait(ul);
    spinner.stop();

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testMsftCameraNoYaml");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
