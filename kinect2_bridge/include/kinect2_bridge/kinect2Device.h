
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

struct KinectParameters {
    bool use_png;
    bool bilateral_filter;
    bool edge_aware_filter;
    bool publishTF;
    int32_t jpeg_quality;
    int32_t png_level;
    int32_t queueSize;
    int32_t reg_dev;
    int32_t depth_dev;
    int32_t worker_threads;
    double maxDepth;
    double minDepth;
    double fps_limit;
    std::string depth_method;
    std::string reg_method;
    std::string calib_path;
    std::string sensor;
    std::string base_name;
    std::string baseNameTF;
};


class Kinect2Device {
private:
    std::vector<int> compressionParams;
    std::string compression16BitExt, compression16BitString;

    cv::Size sizeColor, sizeIr, sizeLowRes;
    cv::Mat color, ir, depth;
    cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr;
    cv::Mat rotation, translation;
    cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;

    std::vector<std::thread> threads;
    std::mutex lockIrDepth, lockColor;
    std::mutex lockSync, lockPub, lockTime, lockStatus;
    std::mutex lockRegLowRes, lockRegHighRes;

    bool publishTF;
    std::thread tfPublisher;

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device* device;
    libfreenect2::SyncMultiFrameListener* listenerColor, *listenerIrDepth;
    libfreenect2::Registration* registration;
    libfreenect2::Freenect2Device::ColorCameraParams colorParams;
    libfreenect2::Freenect2Device::IrCameraParams irParams;
    std::shared_ptr<libfreenect2::Frame> irFrame, depthFrame, colorFrame;

    ros::NodeHandle* nh, priv_nh;

    DepthRegistration* depthRegLowRes, *depthRegHighRes;

    size_t frameColor, frameIrDepth, pubFrameColor, pubFrameIrDepth;
    ros::Time lastColor, lastDepth;

    bool nextColor, nextIrDepth;
    double depthShift, elapsedTimeColor, elapsedTimeIrDepth;
    bool running, deviceActive, clientConnected;

    enum Image {
        IR_SD = 0,
        IR_SD_RECT,

        DEPTH_SD,
        DEPTH_SD_RECT,
        DEPTH_HD,
        DEPTH_QHD,

        COLOR_SD_RECT,
        COLOR_HD,
        COLOR_HD_RECT,
        COLOR_QHD,
        COLOR_QHD_RECT,

        MONO_HD,
        MONO_HD_RECT,
        MONO_QHD,
        MONO_QHD_RECT,

        COUNT
    };

    enum Status {
        UNSUBCRIBED = 0,
        RAW,
        COMPRESSED,
        BOTH
    };

    std::vector<ros::Publisher> imagePubs, compressedPubs;
    ros::Publisher infoHDPub, infoQHDPub, infoIRPub;
    sensor_msgs::CameraInfo infoHD, infoQHD, infoIR;
    std::vector<Status> status;

    int id;
    KinectParameters kinectParams;

public:
    Kinect2Device(libfreenect2::Freenect2Device* _device,
                  int _id,
                  ros::NodeHandle* _nh,
                  KinectParameters _params);
    void start();
    void stop();

private:
    bool initialize();
    bool initRegistration(const std::string& method, const int32_t device, const double maxDepth);
    bool initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png);
    bool initTopics(const int32_t queueSize, const std::string& base_name);
    bool initDevice();
    bool initCalibration(const std::string& calib_path, const std::string& sensor);
    bool loadCalibrationFile(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distortion) const;
    bool loadCalibrationPoseFile(const std::string& filename, cv::Mat& rotation, cv::Mat& translation) const;
    bool loadCalibrationDepthFile(const std::string& filename, double& depthShift) const;
    bool initCameraInfo();
    sensor_msgs::CameraInfo createCameraInfo(const cv::Size& size, const cv::Mat& cameraMatrix, const cv::Mat& distortion, const cv::Mat& rotation, const cv::Mat& projection) const;
    void callbackStatus();
    bool updateStatus();
    void threadDispatcher(const size_t id);
    void receiveIrDepth();
    void receiveColor();
    bool receiveFrames(libfreenect2::SyncMultiFrameListener* listener, libfreenect2::FrameMap& frames);
    std_msgs::Header createHeader(ros::Time& last, ros::Time& other);
    void processIrDepth(const cv::Mat& ir, const cv::Mat& depth, std::vector<cv::Mat>& images, const std::vector<Status>& status);
    void processColor(const cv::Mat& color, std::vector<cv::Mat>& images, const std::vector<Status>& status);
    void publishImages(const std::vector<cv::Mat>& images, const std_msgs::Header& header, const std::vector<Status>& status, const size_t frame, size_t& pubFrame, const size_t begin, const size_t end);
    void createImage(const cv::Mat& image, const std_msgs::Header& header, const Image type, sensor_msgs::Image& msgImage) const;
    void createCompressed(const cv::Mat& image, const std_msgs::Header& header, const Image type, sensor_msgs::CompressedImage& msgImage) const;
    void publishStaticTF();

};























