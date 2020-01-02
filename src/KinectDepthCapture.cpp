//
// Created by kevinskyba on 01.01.20.
//

#include <iostream>
#include <future>
#include <cstring>
#include "KinectDepthCapture.h"

KinectDepthCapture::KinectDepthCapture() {
    if(freenect2_.enumerateDevices() == 0)
    {
        throw std::runtime_error("no device connected");
    }
}

KinectDepthCapture::~KinectDepthCapture() {
    if (dev_ != nullptr) {
        dev_->stop();
    }
}

void KinectDepthCapture::start() {
    std::string serial = freenect2_.getDefaultDeviceSerialNumber();
    // According to implementation Freenect2Device takes over ownership of OpenGLPacketPipeline
    // See https://github.com/OpenKinect/libfreenect2/blob/c67e3b0c6a98b7c82108043bf5fcef18d4c6d713/src/libfreenect2.cpp#L666
    dev_ = std::unique_ptr<libfreenect2::Freenect2Device>(freenect2_.openDevice(serial, new libfreenect2::OpenGLPacketPipeline()));

    freenect_thread_ = std::thread([this]() {
        int types = libfreenect2::Frame::Ir | libfreenect2::Frame::Depth | libfreenect2::Frame::Color;
        libfreenect2::SyncMultiFrameListener listener(types);
        libfreenect2::FrameMap frames;

        this->dev_->setColorFrameListener(&listener);
        this->dev_->setIrAndDepthFrameListener(&listener);

        if (!this->dev_->startStreams(true, true))
            return -1;

        auto registration = libfreenect2::Registration(this->dev_->getIrCameraParams(), this->dev_->getColorCameraParams());

        std::cout << "device serial: " << this->dev_->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << this->dev_->getFirmwareVersion() << std::endl;

        while (true) {
            if (!listener.waitForNewFrame(frames, 1000))
            {
                std::cout << "timeout!" << std::endl;
                std::unique_lock<std::mutex> lk(this->available_mx_);
                this->frame_available_ = false;
                this->available_cv_.notify_one();
                return -1;
            }
            std::unique_lock<std::mutex> lk(this->frame_mx_);

            libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
            libfreenect2::Frame *color = frames[libfreenect2::Frame::Color];

            auto undistortedDepth = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            auto registeredColor = std::make_unique<libfreenect2::Frame>(512, 424, 4);
            registration.apply(color, depth, undistortedDepth.get(), registeredColor.get());

            // Do frame data copy here
            unsigned long dataLength = undistortedDepth->width * undistortedDepth->height;
            auto* dataBuffer = new float[dataLength];
            std::memcpy(dataBuffer, undistortedDepth.get()->data, dataLength * sizeof(float));
            this->frame_ = std::make_shared<DepthFrame>(undistortedDepth->width, undistortedDepth->height, dataBuffer);

            listener.release(frames);

            this->frame_available_ = true;
            this->available_cv_.notify_one();
        }
    });
    freenect_thread_.detach();
}

void KinectDepthCapture::stop() {
    dev_->stop();
}

bool KinectDepthCapture::available() {
    std::mutex cv_m;
    std::unique_lock<std::mutex> lk(cv_m);
    available_cv_.wait_for(lk, std::chrono::seconds (10));
    this->frame_mx_.lock();
    return this->frame_available_;
}

std::shared_ptr<DepthFrame> KinectDepthCapture::getFrame() {
    this->frame_available_ = false;
    this->frame_mx_.unlock();
    return this->frame_;
}