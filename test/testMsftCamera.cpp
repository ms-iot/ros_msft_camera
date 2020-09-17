// Copyright (c) Microsoft Corporation.  All rights reserved.

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <mutex>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

TEST(MsftCameraNode, getImage)
{
    std::mutex waitMutex;
    static std::condition_variable waitForCB;
    ros::NodeHandle node;

    void (*cb)(const sensor_msgs::Image::ConstPtr & image)
        = [](const sensor_msgs::Image::ConstPtr& image)
    {
        EXPECT_EQ("MsftCamera1", image->header.frame_id);
        EXPECT_EQ(720, image->height);
        EXPECT_EQ(1280, image->width);
        EXPECT_EQ("bgra8", image->encoding);
        waitForCB.notify_all();
    };
    std::unique_lock<std::mutex> ul(waitMutex);
    ros::Subscriber sub = node.subscribe("/msft_camera_node/image_raw",
        1,
        cb);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    waitForCB.wait(ul);
    spinner.stop();
}

TEST(MsftCameraNodeTest, getCameraInfo)
{
    std::mutex waitMutex;
    static std::condition_variable waitForCB;
    ros::NodeHandle node;
    void (*cb)(const sensor_msgs::CameraInfo::ConstPtr & info)
        = [](const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        EXPECT_EQ("MsftCamera1", info->header.frame_id);
        // K
        EXPECT_EQ(9, info->K.size());
        EXPECT_NEAR(4827.94, info->K.at(0), 0.001);
        EXPECT_NEAR(0.0, info->K.at(1), 0.001);
        EXPECT_NEAR(1223.5, info->K.at(2), 0.001);
        // D
        EXPECT_EQ(5, info->D.size());
        EXPECT_NEAR(-0.41527, info->D.at(0), 0.001);
        EXPECT_NEAR(0.31874, info->D.at(1), 0.001);
        EXPECT_NEAR(-0.00197, info->D.at(2), 0.001);
        EXPECT_NEAR(0.00071, info->D.at(3), 0.001);
        EXPECT_NEAR(0.0, info->D.at(4), 0.001);

        EXPECT_EQ("plumb_bob", info->distortion_model);

        // width
        EXPECT_EQ(1280, info->width);
        EXPECT_EQ(720, info->height);
        waitForCB.notify_all();
    };

    std::unique_lock<std::mutex> ul(waitMutex);
    ros::Subscriber sub = node.subscribe("/msft_camera_node/camera_info",
        1,
        cb);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    waitForCB.wait(ul);
    spinner.stop();
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "testMsftCamera");

    return RUN_ALL_TESTS();
}
