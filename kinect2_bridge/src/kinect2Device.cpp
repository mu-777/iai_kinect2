

#include"kinect2_bridge/kinect2Device.h"

Kinect2Device::Kinect2Device(libfreenect2::Freenect2Device* _device,
                             int _id,
                             ros::NodeHandle* _nh,
                             KinectParameters _params)
    : sizeColor(1920, 1080),
      sizeIr(512, 424),
      sizeLowRes(sizeColor.width / 2, sizeColor.height / 2),
      priv_nh(priv_nh),
      frameColor(0),
      frameIrDepth(0),
      pubFrameColor(0),
      pubFrameIrDepth(0),
      lastColor(0, 0),
      lastDepth(0, 0),
      nextColor(false),
      nextIrDepth(false),
      depthShift(0),
      running(false),
      deviceActive(false),
      clientConnected(false) {
    color = cv::Mat::zeros(sizeColor, CV_8UC3);
    ir = cv::Mat::zeros(sizeIr, CV_32F);
    depth = cv::Mat::zeros(sizeIr, CV_32F);

    device = _device;
    id = _id;
    nh = _nh;
    kinectParams = _params;
    status.resize(COUNT, UNSUBCRIBED);
}

bool Kinect2Device::initialize() {
    bool isInitialized = true;

    threads.resize(kinectParams.worker_threads);

    if (kinectParams.calib_path.empty() || kinectParams.calib_path.back() != '/') {
        kinectParams.calib_path += '/';
    }

    isInitialized = isInitialized && initDevice();
    isInitialized = isInitialized && initCompression(kinectParams.jpeg_quality,
                                                     kinectParams.png_level,
                                                     kinectParams.use_png);
    isInitialized = isInitialized && initCalibration(kinectParams.calib_path,
                                                     kinectParams.sensor);
    isInitialized = isInitialized && initRegistration(kinectParams.reg_method,
                                                      kinectParams.reg_dev,
                                                      kinectParams.maxDepth);
    isInitialized = isInitialized && initTopics(kinectParams.queueSize,
                                                kinectParams.base_name + "_" + std::to_string(id));

    isInitialized = isInitialized && initCameraInfo();
    return isInitialized;
}

void Kinect2Device::start() {
    if (!initialize()) {
        running = false;
    } else {
        running = true;

        if (kinectParams.publishTF) {
            tfPublisher = std::thread(&Kinect2Device::publishStaticTF, this);
        }

        for (size_t i = 0; i < threads.size(); ++i) {
            threads[i] = std::thread(&Kinect2Device::threadDispatcher, this, i);
        }
    }
}

void Kinect2Device::stop() {
    running = false;

    for (size_t i = 0; i < threads.size(); ++i) {
        threads[i].join();
    }

    if (publishTF) {
        tfPublisher.join();
    }

    device->stop();
    device->close();
    delete listenerIrDepth;
    delete listenerColor;
    delete registration;

    for (size_t i = 0; i < COUNT; ++i) {
        imagePubs[i].shutdown();
        compressedPubs[i].shutdown();
        infoHDPub.shutdown();
        infoQHDPub.shutdown();
        infoIRPub.shutdown();
    }
}

bool Kinect2Device::initRegistration(const std::string& method, const int32_t device, const double maxDepth) {
    DepthRegistration::Method reg;

    if (method == "default") {
        reg = DepthRegistration::DEFAULT;
    } else if (method == "cpu") {
#ifdef DEPTH_REG_CPU
        reg = DepthRegistration::CPU;
#else
        std::cerr << "CPU registration is not available!" << std::endl;
        return -1;
#endif
    } else if (method == "opencl") {
#ifdef DEPTH_REG_OPENCL
        reg = DepthRegistration::OPENCL;
#else
        std::cerr << "OpenCL registration is not available!" << std::endl;
        return -1;
#endif
    } else {
        std::cerr << "Unknown registration method: " << method << std::endl;
        return false;
    }

    depthRegLowRes = DepthRegistration::New(reg);
    depthRegHighRes = DepthRegistration::New(reg);

    bool ret = true;
    ret = ret && depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);
    ret = ret && depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixIr, sizeIr, distortionIr, rotation, translation, 0.5f, maxDepth, device);

    registration = new libfreenect2::Registration(irParams, colorParams);

    return ret;
}

bool Kinect2Device::initCompression(const int32_t jpegQuality, const int32_t pngLevel, const bool use_png) {
    compressionParams.resize(7, 0);
    compressionParams[0] = CV_IMWRITE_JPEG_QUALITY;
    compressionParams[1] = jpegQuality;
    compressionParams[2] = CV_IMWRITE_PNG_COMPRESSION;
    compressionParams[3] = pngLevel;
    compressionParams[4] = CV_IMWRITE_PNG_STRATEGY;
    compressionParams[5] = CV_IMWRITE_PNG_STRATEGY_RLE;
    compressionParams[6] = 0;

    if (use_png) {
        compression16BitExt = ".png";
        compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; png compressed";
    } else {
        compression16BitExt = ".tif";
        compression16BitString = sensor_msgs::image_encodings::TYPE_16UC1 + "; tiff compressed";
    }
    return true;
}

bool Kinect2Device::initTopics(const int32_t queueSize, const std::string& base_name) {
    std::vector<std::string> topics(COUNT);
    topics[IR_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR;
    topics[IR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_IR K2_TOPIC_IMAGE_RECT;

    topics[DEPTH_SD] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH;
    topics[DEPTH_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    topics[DEPTH_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
    topics[DEPTH_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;

    topics[COLOR_SD_RECT] = K2_TOPIC_SD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

    topics[COLOR_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR;
    topics[COLOR_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
    topics[COLOR_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR;
    topics[COLOR_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;

    topics[MONO_HD] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO;
    topics[MONO_HD_RECT] = K2_TOPIC_HD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;
    topics[MONO_QHD] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO;
    topics[MONO_QHD_RECT] = K2_TOPIC_QHD K2_TOPIC_IMAGE_MONO K2_TOPIC_IMAGE_RECT;

    imagePubs.resize(COUNT);
    compressedPubs.resize(COUNT);
    ros::SubscriberStatusCallback cb = boost::bind(&Kinect2Device::callbackStatus, this);

    for (size_t i = 0; i < COUNT; ++i) {
        imagePubs[i] = nh->advertise<sensor_msgs::Image>(base_name + topics[i], queueSize, cb, cb);
        compressedPubs[i] = nh->advertise<sensor_msgs::CompressedImage>(base_name + topics[i] + K2_TOPIC_COMPRESSED, queueSize, cb, cb);
    }
    infoHDPub = nh->advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_HD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoQHDPub = nh->advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_QHD + K2_TOPIC_INFO, queueSize, cb, cb);
    infoIRPub = nh->advertise<sensor_msgs::CameraInfo>(base_name + K2_TOPIC_SD + K2_TOPIC_INFO, queueSize, cb, cb);

    return true;
}

bool Kinect2Device::initDevice() {
    if (device == 0) {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return -1;
    }

    listenerColor = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
    listenerIrDepth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);

    device->setColorFrameListener(listenerColor);
    device->setIrAndDepthFrameListener(listenerIrDepth);

    std::cout << std::endl << "[" << id << "] starting kinect2" << std::endl << std::endl;
    device->start();

    std::cout << std::endl << "[" << id << "] device serial: " << kinectParams.sensor << std::endl;
    std::cout  << "[" << id << "] device firmware: " << device->getFirmwareVersion() << std::endl;

    colorParams = device->getColorCameraParams();
    irParams = device->getIrCameraParams();

    device->stop();

    std::cout << std::endl << "[" << id << "] default ir camera parameters: " << std::endl;
    std::cout << "fx " << irParams.fx << ", fy " << irParams.fy << ", cx " << irParams.cx << ", cy " << irParams.cy << std::endl;
    std::cout << "k1 " << irParams.k1 << ", k2 " << irParams.k2 << ", p1 " << irParams.p1 << ", p2 " << irParams.p2 << ", k3 " << irParams.k3 << std::endl;

    std::cout << std::endl  << "[" << id << "] default color camera parameters: " << std::endl;
    std::cout << "fx " << colorParams.fx << ", fy " << colorParams.fy << ", cx " << colorParams.cx << ", cy " << colorParams.cy << std::endl;

    cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
    distortionColor = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
    cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
    cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
    cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
    cameraMatrixColor.at<double>(2, 2) = 1;

    cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
    distortionIr = cv::Mat::zeros(1, 5, CV_64F);

    cameraMatrixIr.at<double>(0, 0) = irParams.fx;
    cameraMatrixIr.at<double>(1, 1) = irParams.fy;
    cameraMatrixIr.at<double>(0, 2) = irParams.cx;
    cameraMatrixIr.at<double>(1, 2) = irParams.cy;
    cameraMatrixIr.at<double>(2, 2) = 1;

    distortionIr.at<double>(0, 0) = irParams.k1;
    distortionIr.at<double>(0, 1) = irParams.k2;
    distortionIr.at<double>(0, 2) = irParams.p1;
    distortionIr.at<double>(0, 3) = irParams.p2;
    distortionIr.at<double>(0, 4) = irParams.k3;

    rotation = cv::Mat::eye(3, 3, CV_64F);
    translation = cv::Mat::zeros(3, 1, CV_64F);
    translation.at<double>(0) = -0.0520;
    return true;
}

bool Kinect2Device::initCalibration(const std::string& calib_path, const std::string& sensor) {
    std::string calibPath = calib_path + sensor + '/';

    struct stat fileStat;
    bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
    if (calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor)) {
        std::cerr << "using sensor defaults for color intrinsic parameters." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixIr, distortionIr)) {
        std::cerr << "using sensor defaults for ir intrinsic parameters." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation)) {
        std::cerr << "using defaults for rotation and translation." << std::endl;
    }

    if (calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift)) {
        std::cerr << "using defaults for depth shift." << std::endl;
        depthShift = 0.0;
    }

    cameraMatrixLowRes = cameraMatrixColor.clone();
    cameraMatrixLowRes.at<double>(0, 0) /= 2;
    cameraMatrixLowRes.at<double>(1, 1) /= 2;
    cameraMatrixLowRes.at<double>(0, 2) /= 2;
    cameraMatrixLowRes.at<double>(1, 2) /= 2;

    colorParams.fx = cameraMatrixColor.at<double>(0, 0);
    colorParams.fy = cameraMatrixColor.at<double>(1, 1);
    colorParams.cx = cameraMatrixColor.at<double>(0, 2);
    colorParams.cy = cameraMatrixColor.at<double>(1, 2);

    irParams.fx = cameraMatrixIr.at<double>(0, 0);
    irParams.fy = cameraMatrixIr.at<double>(1, 1);
    irParams.cx = cameraMatrixIr.at<double>(0, 2);
    irParams.cy = cameraMatrixIr.at<double>(1, 2);

    irParams.k1 = distortionIr.at<double>(0, 0);
    irParams.k2 = distortionIr.at<double>(0, 1);
    irParams.p1 = distortionIr.at<double>(0, 2);
    irParams.p2 = distortionIr.at<double>(0, 3);
    irParams.k3 = distortionIr.at<double>(0, 4);


    const int mapType = CV_16SC2;
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
    cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
    cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);

    std::cout << std::endl  << "[" << id << "] camera parameters used:" << std::endl
              << "[" << id << "] camera matrix color:" << std::endl << cameraMatrixColor << std::endl
              << "[" << id << "] distortion coefficients color:" << std::endl << distortionColor << std::endl
              << "[" << id << "] camera matrix ir:" << std::endl << cameraMatrixIr << std::endl
              << "[" << id << "] distortion coefficients ir:" << std::endl << distortionIr << std::endl
              << "[" << id << "] rotation:" << std::endl << rotation << std::endl
              << "[" << id << "] translation:" << std::endl << translation << std::endl
              << "[" << id << "] depth shift:" << std::endl << depthShift << std::endl << std::endl;

    return true;
}

bool Kinect2Device::loadCalibrationFile(const std::string& filename, cv::Mat& cameraMatrix, cv::Mat& distortion) const {
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
        fs[K2_CALIB_DISTORTION] >> distortion;
        fs.release();
    } else {
        std::cerr << "can't open calibration file: " << filename << std::endl;
        return false;
    }
    return true;
}

bool Kinect2Device::loadCalibrationPoseFile(const std::string& filename, cv::Mat& rotation, cv::Mat& translation) const {
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_ROTATION] >> rotation;
        fs[K2_CALIB_TRANSLATION] >> translation;
        fs.release();
    } else {
        std::cerr << "can't open calibration pose file: " << filename << std::endl;
        return false;
    }
    return true;
}

bool Kinect2Device::loadCalibrationDepthFile(const std::string& filename, double& depthShift) const {
    cv::FileStorage fs;
    if (fs.open(filename, cv::FileStorage::READ)) {
        fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
        fs.release();
    } else {
        std::cerr << "can't open calibration depth file: " << filename << std::endl;
        return false;
    }
    return true;
}

bool Kinect2Device::initCameraInfo() {
    cv::Mat projColor = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projIr = cv::Mat::zeros(3, 4, CV_64F);
    cv::Mat projLowRes = cv::Mat::zeros(3, 4, CV_64F);

    cameraMatrixColor.copyTo(projColor(cv::Rect(0, 0, 3, 3)));
    cameraMatrixIr.copyTo(projIr(cv::Rect(0, 0, 3, 3)));
    cameraMatrixLowRes.copyTo(projLowRes(cv::Rect(0, 0, 3, 3)));

    infoHD = createCameraInfo(sizeColor, cameraMatrixColor, distortionColor, cv::Mat::eye(3, 3, CV_64F), projColor);
    infoIR = createCameraInfo(sizeIr, cameraMatrixIr, distortionIr, cv::Mat::eye(3, 3, CV_64F), projIr);
    infoQHD = createCameraInfo(sizeLowRes, cameraMatrixLowRes, distortionColor, cv::Mat::eye(3, 3, CV_64F), projLowRes);

    return true;
}

sensor_msgs::CameraInfo Kinect2Device::createCameraInfo(const cv::Size& size, const cv::Mat& cameraMatrix, const cv::Mat& distortion, const cv::Mat& rotation, const cv::Mat& projection) const {
    sensor_msgs::CameraInfo retCameraInfo;

    retCameraInfo.height = size.height;
    retCameraInfo.width = size.width;

    const double* itC = cameraMatrix.ptr<double>(0, 0);
    for (size_t i = 0; i < 9; ++i, ++itC) {
        retCameraInfo.K[i] = *itC;
    }

    const double* itR = rotation.ptr<double>(0, 0);
    for (size_t i = 0; i < 9; ++i, ++itR) {
        retCameraInfo.R[i] = *itR;
    }

    const double* itP = projection.ptr<double>(0, 0);
    for (size_t i = 0; i < 12; ++i, ++itP) {
        retCameraInfo.P[i] = *itP;
    }

    retCameraInfo.distortion_model = "plumb_bob";
    retCameraInfo.D.resize(distortion.cols);
    const double* itD = distortion.ptr<double>(0, 0);
    for (size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD) {
        retCameraInfo.D[i] = *itD;
    }

    return retCameraInfo;
}

void Kinect2Device::callbackStatus() {
    lockStatus.lock();
    clientConnected = updateStatus();

    if (clientConnected && !deviceActive) {
        std::cout << "[kinect2_bridge] client connected. starting device..." << std::endl << std::flush;
        deviceActive = true;
        device->start();
    } else if (!clientConnected && deviceActive) {
        std::cout << "[kinect2_bridge] no clients connected. stopping device..." << std::endl << std::flush;
        deviceActive = false;
        device->stop();
    }
    lockStatus.unlock();
}

bool Kinect2Device::updateStatus() {
    bool any = false;
    for (size_t i = 0; i < COUNT; ++i) {
        Status s = UNSUBCRIBED;
        if (imagePubs[i].getNumSubscribers() > 0) {
            s = RAW;
        }
        if (compressedPubs[i].getNumSubscribers() > 0) {
            s = s == RAW ? BOTH : COMPRESSED;
        }

        status[i] = s;
        any = any || s != UNSUBCRIBED;
    }
    return any || infoHDPub.getNumSubscribers() > 0 || infoQHDPub.getNumSubscribers() > 0 || infoIRPub.getNumSubscribers() > 0;
}

void Kinect2Device::threadDispatcher(const size_t id) {
    const size_t checkFirst = id % 2;
    bool processedFrame = false;
    int oldNice = nice(0);
    oldNice = nice(19 - oldNice);

    for (; running && ros::ok();) {
        processedFrame = false;

        for (size_t i = 0; i < 2; ++i) {
            if (i == checkFirst) {
                if (nextIrDepth && lockIrDepth.try_lock()) {
                    nextIrDepth = false;
                    receiveIrDepth();
                    processedFrame = true;
                }
            } else {
                if (nextColor && lockColor.try_lock()) {
                    nextColor = false;
                    receiveColor();
                    processedFrame = true;
                }
            }
        }

        if (!processedFrame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

void Kinect2Device::receiveIrDepth() {
    libfreenect2::FrameMap frames;
    cv::Mat depth, ir;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

    if (!receiveFrames(listenerIrDepth, frames)) {
        lockIrDepth.unlock();
        return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastDepth, lastColor);

    irFrame = std::shared_ptr<libfreenect2::Frame>(frames[libfreenect2::Frame::Ir]);
    depthFrame = std::shared_ptr<libfreenect2::Frame>(frames[libfreenect2::Frame::Depth]);

    ir = cv::Mat(irFrame->height, irFrame->width, CV_32FC1, irFrame->data);
    depth = cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data);

    frame = frameIrDepth++;
    lockIrDepth.unlock();

    processIrDepth(ir, depth, images, status);

    publishImages(images, header, status, frame, pubFrameIrDepth, IR_SD, COLOR_HD);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeIrDepth += elapsed;
    lockTime.unlock();
}

void Kinect2Device::receiveColor() {
    libfreenect2::FrameMap frames;
    cv::Mat color;
    std_msgs::Header header;
    std::vector<cv::Mat> images(COUNT);
    std::vector<Status> status = this->status;
    size_t frame;

    if (!receiveFrames(listenerColor, frames)) {
        lockColor.unlock();
        return;
    }
    double now = ros::Time::now().toSec();

    header = createHeader(lastColor, lastDepth);

    colorFrame = std::shared_ptr<libfreenect2::Frame>(frames[libfreenect2::Frame::Color]);

    color = cv::Mat(colorFrame->height, colorFrame->width, CV_8UC4, colorFrame->data);

    frame = frameColor++;
    lockColor.unlock();

    processColor(color, images, status);

    publishImages(images, header, status, frame, pubFrameColor, COLOR_HD, COUNT);

    double elapsed = ros::Time::now().toSec() - now;
    lockTime.lock();
    elapsedTimeColor += elapsed;
    lockTime.unlock();
}

bool Kinect2Device::receiveFrames(libfreenect2::SyncMultiFrameListener* listener, libfreenect2::FrameMap& frames) {
    bool newFrames = false;
    for (; !newFrames;) {
#ifdef LIBFREENECT2_THREADING_STDLIB
        newFrames = listener->waitForNewFrame(frames, 1000);
#else
        newFrames = true;
        listener->waitForNewFrame(frames);
#endif
        if (!deviceActive || !running || !ros::ok()) {
            if (newFrames) {
                listener->release(frames);
            }
            return false;
        }
    }
    return true;
}

std_msgs::Header Kinect2Device::createHeader(ros::Time& last, ros::Time& other) {
    ros::Time timestamp = ros::Time::now();
    lockSync.lock();
    if (other.isZero()) {
        last = timestamp;
    } else {
        timestamp = other;
        other = ros::Time(0, 0);
    }
    lockSync.unlock();

    std_msgs::Header header;
    header.seq = 0;
    header.stamp = timestamp;
    header.frame_id = K2_TF_RGB_OPT_FRAME;
    return header;
}

void Kinect2Device::processIrDepth(const cv::Mat& ir, const cv::Mat& depth, std::vector<cv::Mat>& images, const std::vector<Status>& status) {
    // COLOR registered to depth
    if (status[COLOR_SD_RECT]) {
        if (!colorFrame) {
            images[COLOR_SD_RECT] = cv::Mat::zeros(sizeIr, CV_8UC3);
        } else {
            std::shared_ptr<libfreenect2::Frame> tmpColor, tmpDepth;
            cv::Mat tmp;
            libfreenect2::Frame undistorted(sizeIr.width, sizeIr.height, 4), registered(sizeIr.width, sizeIr.height, 4);
            tmpColor = colorFrame;
            tmpDepth = depthFrame;
            registration->apply(tmpColor.get(), tmpDepth.get(), &undistorted, &registered);
            cv::flip(cv::Mat(sizeIr, CV_8UC4, registered.data), tmp, 1);
            cv::cvtColor(tmp, images[COLOR_SD_RECT], CV_BGRA2BGR);
        }
    }

    // IR
    if (status[IR_SD] || status[IR_SD_RECT]) {
        ir.convertTo(images[IR_SD], CV_16U);
        cv::flip(images[IR_SD], images[IR_SD], 1);
    }
    if (status[IR_SD_RECT]) {
        cv::remap(images[IR_SD], images[IR_SD_RECT], map1Ir, map2Ir, cv::INTER_AREA);
    }

    // DEPTH
    cv::Mat depthShifted;
    if (status[DEPTH_SD]) {
        depth.convertTo(images[DEPTH_SD], CV_16U, 1);
        cv::flip(images[DEPTH_SD], images[DEPTH_SD], 1);
    }
    if (status[DEPTH_SD_RECT] || status[DEPTH_QHD] || status[DEPTH_HD]) {
        depth.convertTo(depthShifted, CV_16U, 1, depthShift);
        cv::flip(depthShifted, depthShifted, 1);
    }
    if (status[DEPTH_SD_RECT]) {
        cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
    }
    if (status[DEPTH_QHD]) {
        lockRegLowRes.lock();
        depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD]);
        lockRegLowRes.unlock();
    }
    if (status[DEPTH_HD]) {
        lockRegHighRes.lock();
        depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);
        lockRegHighRes.unlock();
    }
}

void Kinect2Device::processColor(const cv::Mat& color, std::vector<cv::Mat>& images, const std::vector<Status>& status) {
    // COLOR
    if (status[COLOR_HD] || status[COLOR_HD_RECT] || status[COLOR_QHD] || status[COLOR_QHD_RECT] ||
            status[MONO_HD] || status[MONO_HD_RECT] || status[MONO_QHD] || status[MONO_QHD_RECT]) {
        cv::Mat tmp;
        cv::flip(color, tmp, 1);
        cv::cvtColor(tmp, images[COLOR_HD], CV_BGRA2BGR);
    }
    if (status[COLOR_HD_RECT] || status[MONO_HD_RECT]) {
        cv::remap(images[COLOR_HD], images[COLOR_HD_RECT], map1Color, map2Color, cv::INTER_AREA);
    }
    if (status[COLOR_QHD] || status[MONO_QHD]) {
        cv::resize(images[COLOR_HD], images[COLOR_QHD], sizeLowRes, 0, 0, cv::INTER_AREA);
    }
    if (status[COLOR_QHD_RECT] || status[MONO_QHD_RECT]) {
        cv::remap(images[COLOR_HD], images[COLOR_QHD_RECT], map1LowRes, map2LowRes, cv::INTER_AREA);
    }

    // MONO
    if (status[MONO_HD]) {
        cv::cvtColor(images[COLOR_HD], images[MONO_HD], CV_BGR2GRAY);
    }
    if (status[MONO_HD_RECT]) {
        cv::cvtColor(images[COLOR_HD_RECT], images[MONO_HD_RECT], CV_BGR2GRAY);
    }
    if (status[MONO_QHD]) {
        cv::cvtColor(images[COLOR_QHD], images[MONO_QHD], CV_BGR2GRAY);
    }
    if (status[MONO_QHD_RECT]) {
        cv::cvtColor(images[COLOR_QHD_RECT], images[MONO_QHD_RECT], CV_BGR2GRAY);
    }
}

void Kinect2Device::publishImages(const std::vector<cv::Mat>& images, const std_msgs::Header& header, const std::vector<Status>& status, const size_t frame, size_t& pubFrame, const size_t begin, const size_t end) {
    std::vector<sensor_msgs::ImagePtr> imageMsgs(COUNT);
    std::vector<sensor_msgs::CompressedImagePtr> compressedMsgs(COUNT);
    sensor_msgs::CameraInfoPtr infoHDMsg,  infoQHDMsg,  infoIRMsg;
    std_msgs::Header _header = header;

    if (begin < COLOR_HD) {
        _header.frame_id = kinectParams.baseNameTF + K2_TF_IR_OPT_FRAME;

        infoIRMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoIRMsg = infoIR;
        infoIRMsg->header = _header;
    } else {
        _header.frame_id = kinectParams.baseNameTF + K2_TF_RGB_OPT_FRAME;

        infoHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoHDMsg = infoHD;
        infoHDMsg->header = _header;

        infoQHDMsg = sensor_msgs::CameraInfoPtr(new sensor_msgs::CameraInfo);
        *infoQHDMsg = infoQHD;
        infoQHDMsg->header = _header;

    }

    for (size_t i = begin; i < end; ++i) {
        if (i < DEPTH_HD || i == COLOR_SD_RECT) {
            _header.frame_id = kinectParams.baseNameTF + K2_TF_IR_OPT_FRAME;
        } else {
            _header.frame_id = kinectParams.baseNameTF + K2_TF_RGB_OPT_FRAME;
        }

        switch (status[i]) {
            case UNSUBCRIBED:
                break;
            case RAW:
                imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
                createImage(images[i], _header, Image(i), *imageMsgs[i]);
                break;
            case COMPRESSED:
                compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
                createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
                break;
            case BOTH:
                imageMsgs[i] = sensor_msgs::ImagePtr(new sensor_msgs::Image);
                compressedMsgs[i] = sensor_msgs::CompressedImagePtr(new sensor_msgs::CompressedImage);
                createImage(images[i], _header, Image(i), *imageMsgs[i]);
                createCompressed(images[i], _header, Image(i), *compressedMsgs[i]);
                break;
        }
    }

    while (frame != pubFrame) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    lockPub.lock();
    for (size_t i = begin; i < end; ++i) {
        switch (status[i]) {
            case UNSUBCRIBED:
                break;
            case RAW:
                imagePubs[i].publish(imageMsgs[i]);
                break;
            case COMPRESSED:
                compressedPubs[i].publish(compressedMsgs[i]);
                break;
            case BOTH:
                imagePubs[i].publish(imageMsgs[i]);
                compressedPubs[i].publish(compressedMsgs[i]);
                break;
        }
    }

    if (begin < COLOR_HD) {
        if (infoIRPub.getNumSubscribers() > 0) {
            infoIRPub.publish(infoIRMsg);
        }
    } else {
        if (infoHDPub.getNumSubscribers() > 0) {
            infoHDPub.publish(infoHDMsg);
        }
        if (infoQHDPub.getNumSubscribers() > 0) {
            infoQHDPub.publish(infoQHDMsg);
        }
    }

    ++pubFrame;
    lockPub.unlock();
}

void Kinect2Device::createImage(const cv::Mat& image, const std_msgs::Header& header, const Image type, sensor_msgs::Image& msgImage) const {
    size_t step, size;
    step = image.cols * image.elemSize();
    size = image.rows * step;

    switch (type) {
        case IR_SD:
        case IR_SD_RECT:
        case DEPTH_SD:
        case DEPTH_SD_RECT:
        case DEPTH_HD:
        case DEPTH_QHD:
            msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            break;
        case COLOR_SD_RECT:
        case COLOR_HD:
        case COLOR_HD_RECT:
        case COLOR_QHD:
        case COLOR_QHD_RECT:
            msgImage.encoding = sensor_msgs::image_encodings::BGR8;
            break;
        case MONO_HD:
        case MONO_HD_RECT:
        case MONO_QHD:
        case MONO_QHD_RECT:
            msgImage.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
            break;
        case COUNT:
            return;
    }

    msgImage.header = header;
    msgImage.height = image.rows;
    msgImage.width = image.cols;
    msgImage.is_bigendian = false;
    msgImage.step = step;
    msgImage.data.resize(size);
    memcpy(msgImage.data.data(), image.data, size);
}


void Kinect2Device::createCompressed(const cv::Mat& image, const std_msgs::Header& header, const Image type, sensor_msgs::CompressedImage& msgImage) const {
    msgImage.header = header;

    switch (type) {
        case IR_SD:
        case IR_SD_RECT:
        case DEPTH_SD:
        case DEPTH_SD_RECT:
        case DEPTH_HD:
        case DEPTH_QHD:
            msgImage.format = compression16BitString;
            cv::imencode(compression16BitExt, image, msgImage.data, compressionParams);
            break;
        case COLOR_SD_RECT:
        case COLOR_HD:
        case COLOR_HD_RECT:
        case COLOR_QHD:
        case COLOR_QHD_RECT:
            msgImage.format = sensor_msgs::image_encodings::BGR8 + "; jpeg compressed bgr8";
            cv::imencode(".jpg", image, msgImage.data, compressionParams);
            break;
        case MONO_HD:
        case MONO_HD_RECT:
        case MONO_QHD:
        case MONO_QHD_RECT:
            msgImage.format = sensor_msgs::image_encodings::TYPE_8UC1 + "; jpeg compressed ";
            cv::imencode(".jpg", image, msgImage.data, compressionParams);
            break;
        case COUNT:
            return;
    }
}

void Kinect2Device::publishStaticTF() {
    tf::TransformBroadcaster broadcaster;
    tf::StampedTransform stColorOpt, stIrOpt;
    ros::Time now = ros::Time::now();

    tf::Matrix3x3 rot(rotation.at<double>(0, 0), rotation.at<double>(0, 1), rotation.at<double>(0, 2),
                      rotation.at<double>(1, 0), rotation.at<double>(1, 1), rotation.at<double>(1, 2),
                      rotation.at<double>(2, 0), rotation.at<double>(2, 1), rotation.at<double>(2, 2));

    tf::Quaternion qZero;
    qZero.setRPY(0, 0, 0);
    tf::Vector3 trans(translation.at<double>(0), translation.at<double>(1), translation.at<double>(2));
    tf::Vector3 vZero(0, 0, 0);
    tf::Transform tIr(rot, trans), tZero(qZero, vZero);

    stColorOpt = tf::StampedTransform(tZero, now, kinectParams.baseNameTF + K2_TF_LINK, kinectParams.baseNameTF + K2_TF_RGB_OPT_FRAME);
    stIrOpt = tf::StampedTransform(tIr, now, kinectParams.baseNameTF + K2_TF_RGB_OPT_FRAME, kinectParams.baseNameTF + K2_TF_IR_OPT_FRAME);

    for (; running && ros::ok();) {
        now = ros::Time::now();
        stColorOpt.stamp_ = now;
        stIrOpt.stamp_ = now;

        broadcaster.sendTransform(stColorOpt);
        broadcaster.sendTransform(stIrOpt);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

