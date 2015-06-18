
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/stat.h>

#include <opencv2/opencv.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>

#include <compressed_depth_image_transport/compression_common.h>


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/config.h>
#include <libfreenect2/registration.h>

#include <kinect2_bridge/kinect2_definitions.h>
#include <kinect2_registration/kinect2_registration.h>

#include "kinect2_bridge/kinect2Device.h"


class Kinect2Bridge {
private:
    ros::NodeHandle nh, priv_nh;
    double mDeltaT;
    bool mIsRunning;

    std::vector<Kinect2Device*> mKinect2Devices;
    std::thread mMainThread;
    libfreenect2::Freenect2 mFreenect2;



public:
    Kinect2Bridge(const ros::NodeHandle& nh = ros::NodeHandle(),
                  const ros::NodeHandle& priv_nh = ros::NodeHandle("~"));
    void start();
    void stop();

private:
    bool initialize();
    bool isExistDevice(std::string& sensor);
    libfreenect2::Freenect2Device* createDevice(std::string& sensor,
                                                libfreenect2::PacketPipeline* packetPipelines);
    libfreenect2::PacketPipeline* createPipeline(const std::string& method,
                                                 const int32_t device,
                                                 const bool bilateral_filter,
                                                 const bool edge_aware_filter,
                                                 const double minDepth,
                                                 const double maxDepth);
    void main();

};























