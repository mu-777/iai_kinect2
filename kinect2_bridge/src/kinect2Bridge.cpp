

#include"kinect2_bridge/kinect2Bridge.h"

Kinect2Bridge::Kinect2Bridge(const ros::NodeHandle& nh, const ros::NodeHandle& priv_nh)
    : nh(nh),
      priv_nh(priv_nh),
      mIsRunning(false) {
}

void Kinect2Bridge::start() {
    if (!initialize()) {
        mIsRunning =  false;
    } else {
        mIsRunning = true;
        std::for_each(mKinect2Devices.begin(), mKinect2Devices.end(), [](Kinect2Device * device) {
            device->start();
        });

        mMainThread = std::thread(&Kinect2Bridge::main, this);
    }
}


void Kinect2Bridge::stop() {
    mIsRunning = false;

    std::for_each(mKinect2Devices.begin(), mKinect2Devices.end(), [](Kinect2Device * device) {
        device->stop();
    });

    mMainThread.join();

    nh.shutdown();
}

bool Kinect2Bridge::initialize() {
    int num_of_sensors;
    std::vector<std::string> sensor_serial_ids;
    KinectParameters params;

    std::string depthDefault = "cpu";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_OPENCL
    regDefault = "opencl";
#endif

    priv_nh.param("base_name", params.base_name, std::string(K2_DEFAULT_NS));
    priv_nh.param("num_of_sensors", num_of_sensors, 0);
    priv_nh.param("fps_limit", params.fps_limit, -1.0);
    priv_nh.param("calib_path", params.calib_path, std::string(K2_CALIB_PATH));
    priv_nh.param("use_png", params.use_png, false);
    priv_nh.param("jpeg_quality", params.jpeg_quality, 90);
    priv_nh.param("png_level", params.png_level, 1);
    priv_nh.param("depth_method", params.depth_method, depthDefault);
    priv_nh.param("depth_device", params.depth_dev, -1);
    priv_nh.param("reg_method", params.reg_method, regDefault);
    priv_nh.param("reg_devive", params.reg_dev, -1);
    priv_nh.param("max_depth", params.maxDepth, 12.0);
    priv_nh.param("min_depth", params.minDepth, 0.1);
    priv_nh.param("queue_size", params.queueSize, 2);
    priv_nh.param("bilateral_filter", params.bilateral_filter, true);
    priv_nh.param("edge_aware_filter", params.edge_aware_filter, true);
    priv_nh.param("publish_tf", params.publishTF, false);
    priv_nh.param("base_name_tf", params.baseNameTF, params.base_name);
    priv_nh.param("worker_threads", params.worker_threads, 4);
    priv_nh.getParam("sensor_serial_ids", sensor_serial_ids);

    params.worker_threads = std::min(std::max((int)std::thread::hardware_concurrency() / num_of_sensors, 1),
                                     (int)params.worker_threads);

    std::cout << "parameter:" << std::endl
              << "        base_name: " << params.base_name << std::endl
              //<< "          sensors: " << sensor_serial_ids << std::endl
              << "        fps_limit: " << params.fps_limit << std::endl
              << "       calib_path: " << params.calib_path << std::endl
              << "          use_png: " << (params.use_png ? "true" : "false") << std::endl
              << "     jpeg_quality: " << params.jpeg_quality << std::endl
              << "        png_level: " << params.png_level << std::endl
              << "     depth_method: " << params.depth_method << std::endl
              << "     depth_device: " << params.depth_dev << std::endl
              << "       reg_method: " << params.reg_method << std::endl
              << "       reg_devive: " << params.reg_dev << std::endl
              << "        max_depth: " << params.maxDepth << std::endl
              << "        min_depth: " << params.minDepth << std::endl
              << "       queue_size: " << params.queueSize << std::endl
              << " bilateral_filter: " << (params.bilateral_filter ? "true" : "false") << std::endl
              << "edge_aware_filter: " << (params.edge_aware_filter ? "true" : "false") << std::endl
              << "       publish_tf: " << (params.publishTF ? "true" : "false") << std::endl
              << "     base_name_tf: " << params.baseNameTF << std::endl
              << "   worker_threads: " << params.worker_threads << std::endl << std::endl;

    mDeltaT = params.fps_limit > 0 ? 1.0 / params.fps_limit : 0.0;

    bool ret = true;

    mKinect2Devices.resize(num_of_sensors);

    for (int i = 0; i < num_of_sensors; i++) {
        if (isExistDevice(sensor_serial_ids[i])) {
            params.sensor = sensor_serial_ids[i];
            mKinect2Devices[i] = new Kinect2Device(createDevice(sensor_serial_ids[i],
                                                                createPipeline(params.depth_method,
                                                                               params.depth_dev,
                                                                               params.bilateral_filter,
                                                                               params.edge_aware_filter,
                                                                               params.minDepth,
                                                                               params.maxDepth)),
                                                   i, &nh, params);
        }
    }

    return ret;
}

bool Kinect2Bridge::isExistDevice(std::string& sensor) {
    const int numOfDevs = mFreenect2.enumerateDevices();

    if (numOfDevs <= 0) {
        std::cerr << "Error: no Kinect2 devices found!" << std::endl;
        return false;
    }

    if (sensor.empty()) {
        sensor = mFreenect2.getDefaultDeviceSerialNumber();
    }

    for (int i = 0; i < numOfDevs; ++i) {
        const std::string& s = mFreenect2.getDeviceSerialNumber(i);
        if (s == sensor) {
            return true;
        }
    }

    return false;
}

libfreenect2::Freenect2Device* Kinect2Bridge::createDevice(std::string& sensor,
                                                           libfreenect2::PacketPipeline* packetPipeline) {

    if (sensor.empty()) {
        sensor = mFreenect2.getDefaultDeviceSerialNumber();
    }

    libfreenect2::Freenect2Device* device = mFreenect2.openDevice(sensor, packetPipeline);

    if (device == 0) {
        std::cout << "no device connected or failure opening the default one!" << std::endl;
        return device;
    }

    return device;
}

libfreenect2::PacketPipeline* Kinect2Bridge::createPipeline(const std::string& method, const int32_t device, const bool bilateral_filter, const bool edge_aware_filter, const double minDepth, const double maxDepth) {
    libfreenect2::PacketPipeline* retPacketPipeline;

    if (method == "cpu") {
        retPacketPipeline = new libfreenect2::CpuPacketPipeline();
    } else if (method == "opencl") {
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        retPacketPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#else
        std::cerr << "OpenCL depth processing is not available!" << std::endl;
        retPacketPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    } else if (method == "opengl") {
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        retPacketPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        std::cerr << "OpenGL depth processing is not available!" << std::endl;
        retPacketPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    } else {// include method == "default"
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        retPacketPipeline = new libfreenect2::OpenCLPacketPipeline(device);
#elif defined(LIBFREENECT2_WITH_OPENGL_SUPPORT)
        retPacketPipeline = new libfreenect2::OpenGLPacketPipeline();
#else
        retPacketPipeline = new libfreenect2::CpuPacketPipeline();
#endif
    }


    libfreenect2::DepthPacketProcessor::Config config;
    config.EnableBilateralFilter = bilateral_filter;
    config.EnableEdgeAwareFilter = edge_aware_filter;
    config.MinDepth = minDepth;
    config.MaxDepth = maxDepth;
    retPacketPipeline->getDepthPacketProcessor()->setConfiguration(config);
    return retPacketPipeline;
}


void Kinect2Bridge::main() {
    //    std::cout << "[kinect2_bridge] waiting for clients to connect" << std::endl << std::endl;
    //    double nextFrame = ros::Time::now().toSec() + mDeltaT;
    //    double fpsTime = ros::Time::now().toSec();
    //    size_t oldFrameIrDepth = 0, oldFrameColor = 0;
    //    nextColor = true;
    //    nextIrDepth = true;

    for (; mIsRunning && ros::ok();) {
        //        if (!deviceActive) {
        //            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //            fpsTime =  ros::Time::now().toSec();
        //            nextFrame = fpsTime + mDeltaT;
        //            continue;
        //        }

        //        double now = ros::Time::now().toSec();

        //        if (now - fpsTime >= 3.0) {
        //            fpsTime = now - fpsTime;
        //            size_t framesIrDepth = frameIrDepth - oldFrameIrDepth;
        //            size_t framesColor = frameColor - oldFrameColor;
        //            oldFrameIrDepth = frameIrDepth;
        //            oldFrameColor = frameColor;

        //            lockTime.lock();
        //            double tColor = elapsedTimeColor;
        //            double tDepth = elapsedTimeIrDepth;
        //            elapsedTimeColor = 0;
        //            elapsedTimeIrDepth = 0;
        //            lockTime.unlock();

        //            std::cout << "[kinect2_bridge] depth processing: ~" << framesIrDepth / tDepth << "Hz (" << (tDepth / framesIrDepth) * 1000 << "ms) publishing rate: ~" << framesIrDepth / fpsTime << "Hz" << std::endl
        //                      << "[kinect2_bridge] color processing: ~" << framesColor / tColor << "Hz (" << (tColor / framesColor) * 1000 << "ms) publishing rate: ~" << framesColor / fpsTime << "Hz" << std::endl << std::flush;
        //            fpsTime = now;
        //        }

        //        if (now >= nextFrame) {
        //            nextColor = true;
        //            nextIrDepth = true;
        //            nextFrame += mDeltaT;
        //        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));

        //        if (!deviceActive) {
        //            oldFrameIrDepth = frameIrDepth;
        //            oldFrameColor = frameColor;
        //            lockTime.lock();
        //            elapsedTimeColor = 0;
        //            elapsedTimeIrDepth = 0;
        //            lockTime.unlock();
        //            continue;
        //        }
    }
}

