#include <iostream>
#include <cstdlib>
#include <signal.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

bool protonect_shutdown = false;

int main(int argc, char *argv[]) {

    libfreenect2::Freenect2 freenect2;
    libfreenect2::Freenect2Device *dev = 0;
    libfreenect2::PacketPipeline *pipeline = 0;

    std::string serial = "";

    bool viewer_enabled = true;
    bool enable_rgb = true;
    bool enable_depth = true;
    int deviceId = -1;
    size_t framemax = -1;

    // Initialize and Discover Devices

    if(freenect2.enumerateDevices() == 0) {
        std::cout << "no device connected!" << std::endl;
        return -1;
    }

    if (serial == "") {
        serial = freenect2.getDefaultDeviceSerialNumber();
    }

    // Open device

    if(pipeline) {
        dev = freenect2.openDevice(serial, pipeline);
    } else {
        dev = freenect2.openDevice(serial);
    }

    // set frame listeners

    int types = 0;
    if (enable_rgb)
        types |= libfreenect2::Frame::Color;
    if (enable_depth)
        types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;

    libfreenect2::SyncMultiFrameListener listener(types);
    libfreenect2::FrameMap frames;

    dev->setColorFrameListener(&listener);
    dev->setIrAndDepthFrameListener(&listener);

    // start the device

    if (enable_rgb && enable_depth) {
        if (!dev->start())
            return -1;
    } else {
        if (!dev->startStreams(enable_rgb, enable_depth))
            return -1;
    }

    std::cout << "starting... " << std::endl;
    std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

    // Recieve image frames

    libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());

    libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);

    size_t framecount = 0;

    while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax)) {

        if (!listener.waitForNewFrame(frames, 10*1000)) {
            std::cout << "timeout!" << std::endl;
            return -1;
        }

        libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
        libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
        libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];

        if (enable_rgb && enable_depth) {
            registration->apply(rgb, depth, &undistorted, &registered);
        }

        framecount++;

        listener.release(frames);
    }

    dev->stop();
    dev->close();

    delete registration;

    return 0;
}
