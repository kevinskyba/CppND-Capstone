//
// Created by kevinskyba on 01.01.20.
//

#include <iostream>
#include <fstream>
#include <future>
#include <cstring>
#include <sstream>
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

    // Start will start a thread
    freenect_thread_ = std::thread([this]() {

        // Stuff required by libfreenect2
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
            // We wait 10 seconds for a new frame
            if (!listener.waitForNewFrame(frames, 1000))
            {
                std::cout << "timeout!" << std::endl;

                // frame_available_ is made thread-safe by a mutex. frame_available_ is used by available() and
                // getFrame() to access the current frame. With mutex we can make sure that there are no invalid reads
                // or writes.
                std::unique_lock<std::mutex> lk(this->available_mx_);
                this->frame_available_ = false;
                this->available_cv_.notify_one();
                return -1;
            }

            // Using frame_mx_ we prevent parallel access to the frame_ variable.
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
            this->available_cv_.notify_one(); // Notify available() to not wait anymore.
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

    // available() is locked because it is expected that after available() a call to getFrame() will be done.
    this->frame_mx_.lock();
    return this->frame_available_;
}

std::shared_ptr<DepthFrame> KinectDepthCapture::getFrame() {
    this->frame_available_ = false;

    // Unlock the mutex previously locked by available()
    this->frame_mx_.unlock();
    return this->frame_;
}

void KinectDepthCapture::save(const std::string& path) {
    std::lock_guard lg(this->frame_mx_);

    /**
     * This will save a DepthFrame to a file of .frame format. .frame is a very simple custom written format
     * with no overhead.
     *
     * -------------------------------------------------------------------------------------------
     * |              |              |
     * |    width     |    height    |             data
     * |   (8 byte)   |   (8 byte)   |   (width * height * 4 byte)         ...
     * |              |              |
     * -------------------------------------------------------------------------------------------
     */

    std::stringstream ss;
    std::time_t result = std::time(nullptr);
    ss << result;
    std::string time = ss.str();

    std::ofstream fout;
    fout.open(path + "/" + time + ".frame", std::ios::binary | std::ios::out);

    /**
     * NOTE! For this project the code is expected to run on a 64 bit system. So size_t will always be 8 byte.
     */
    fout.write(reinterpret_cast<const char*>(&this->frame_->width), sizeof(size_t));
    fout.write(reinterpret_cast<const char*>(&this->frame_->height), sizeof(size_t));
    for (int i = 0; i < this->frame_->width * this->frame_->height; i++) {
        fout.write(reinterpret_cast<const char*>(&this->frame_->getData()[i]), sizeof(float));
    }

    fout.close();
}
