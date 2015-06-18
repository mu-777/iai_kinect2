/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/stat.h>

#include <ros/ros.h>

#include "kinect2_bridge/kinect2Bridge.h"

class Kinect2BridgeNodelet : public nodelet::Nodelet {
private:
    std::thread kinect2BridgeThread;
    Kinect2Bridge* pKinect2Bridge;

public:
    Kinect2BridgeNodelet() : Nodelet(), pKinect2Bridge(NULL) {
    }

    ~Kinect2BridgeNodelet() {
        if (pKinect2Bridge) {
            pKinect2Bridge->stop();
            delete pKinect2Bridge;
        }
    }

    virtual void onInit() {
        pKinect2Bridge = new Kinect2Bridge(getNodeHandle(), getPrivateNodeHandle());
        pKinect2Bridge->start();
    }
};

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(Kinect2BridgeNodelet, nodelet::Nodelet)

void helpOption(const std::string& name, const std::string& stype, const std::string& value, const std::string& desc) {
    std::cout  << '_' << name << ":=<" << stype << '>' << std::endl
               << "    default: " << value << std::endl
               << "    info:    " << desc << std::endl;
}

void help(const std::string& path) {
    std::string depthMethods = "cpu";
    std::string depthDefault = "cpu";
    std::string regMethods = "default";
    std::string regDefault = "default";

#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
    depthMethods += ", opengl";
    depthDefault = "opengl";
#endif
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
    depthMethods += ", opencl";
    depthDefault = "opencl";
#endif
#ifdef DEPTH_REG_CPU
    regMethods += ", cpu";
#endif
#ifdef DEPTH_REG_OPENCL
    regMethods += ", opencl";
    regDefault = "opencl";
#endif

    std::cout << path << " [_options:=value]" << std::endl;
    helpOption("base_name",         "string", K2_DEFAULT_NS,  "set base name for all topics");
    helpOption("sensor",            "double", "-1.0",         "serial of the sensor to use");
    helpOption("fps_limit",         "double", "-1.0",         "limit the frames per second");
    helpOption("calib_path",        "string", K2_CALIB_PATH,  "path to the calibration files");
    helpOption("use_png",           "bool",   "false",        "Use PNG compression instead of TIFF");
    helpOption("jpeg_quality",      "int",    "90",           "JPEG quality level from 0 to 100");
    helpOption("png_level",         "int",    "1",            "PNG compression level from 0 to 9");
    helpOption("depth_method",      "string", depthDefault,   "Use specific depth processing: " + depthMethods);
    helpOption("depth_device",      "int",    "-1",           "openCL device to use for depth processing");
    helpOption("reg_method",        "string", regDefault,     "Use specific depth registration: " + regMethods);
    helpOption("reg_devive",        "int",    "-1",           "openCL device to use for depth registration");
    helpOption("max_depth",         "double", "12.0",         "max depth value");
    helpOption("min_depth",         "double", "0.1",          "min depth value");
    helpOption("queue_size",        "int",    "2",            "queue size of publisher");
    helpOption("bilateral_filter",  "bool",   "true",         "enable bilateral filtering of depth images");
    helpOption("edge_aware_filter", "bool",   "true",         "enable edge aware filtering of depth images");
    helpOption("publish_tf",        "bool",   "false",        "publish static tf transforms for camera");
    helpOption("base_name_tf",      "string", "as base_name", "base name for the tf frames");
    helpOption("worker_threads",    "int",    "4",            "number of threads used for processing the images");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kinect2_bridge");


    for (int argI = 1; argI < argc; ++argI) {
        std::string arg(argv[argI]);

        if (arg == "--help" || arg == "--h" || arg == "-h" || arg == "-?" || arg == "--?") {
            help(argv[0]);
            ros::shutdown();
            return 0;
        } else {
            std::cerr << "Unknown argument: " << arg << std::endl;
            return -1;
        }
    }

    if (!ros::ok()) {
        std::cerr << "ros::ok failed!" << std::endl;
        return -1;
    }

    Kinect2Bridge kinect2;
    kinect2.start();

    ros::spin();

    kinect2.stop();

    ros::shutdown();
    return 0;
}
