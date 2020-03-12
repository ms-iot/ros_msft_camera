// Copyright (c) Microsoft Corporation.  All rights reserved.

#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <gtest/gtest.h>

TEST(WinCameraNode, getImage)
{
    ros::NodeHandle node;
    static std::mutex gWaitMutex;
    void (*cb)(const sensor_msgs::Image::ConstPtr & image) = [](const sensor_msgs::Image::ConstPtr& image)
    {
        EXPECT_EQ("WinCamera2", image->header.frame_id);
        EXPECT_EQ(720, image->height);
        EXPECT_EQ(1280, image->width);
        EXPECT_EQ("bgra8", image->encoding);
        gWaitMutex.unlock();
    };

    gWaitMutex.lock();
    ros::Subscriber sub = node.subscribe("/win_camera_no_yaml/image_raw",
        1,
        cb);
    ros::Rate r(30.0);

    while (!gWaitMutex.try_lock())
    {
        ros::spinOnce();
        r.sleep();
    }
    gWaitMutex.unlock();
}

TEST(WinCameraNodeTest, getCameraInfo)
{
    ros::NodeHandle node;
    static std::mutex gWaitMutex;
    void (*cb)(const sensor_msgs::CameraInfo::ConstPtr & info)
        = [](const sensor_msgs::CameraInfo::ConstPtr& info)
    {
        EXPECT_EQ("WinCamera2", info->header.frame_id);
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
        gWaitMutex.unlock();

    };
    ros::Subscriber sub = node.subscribe("/win_camera_no_yaml/camera_info",
        1,
        cb);
    ros::Rate r(10.0);
    while (sub.getNumPublishers() == 0)
    {
        r.sleep();
    }
    while (!gWaitMutex.try_lock())
    {
        ros::spinOnce();
        r.sleep();
    }
    gWaitMutex.unlock();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "testWinCameraNoYaml");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
