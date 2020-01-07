//
// Created by kevinskyba on 01.01.20.
//

#ifndef CPPND_CAPSTONE_KINECTDEPTHCAPTURE_H
#define CPPND_CAPSTONE_KINECTDEPTHCAPTURE_H

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <thread>
#include <condition_variable>

#include "DepthCapture.h"

/**
 * KinectDepthCapture is the implementation of DepthCapture which does require a Kinect V2 (Xbox One).
 * It will connect to the device using libfreenect2.
 */
class KinectDepthCapture : public DepthCapture {
public:

    KinectDepthCapture();
    ~KinectDepthCapture();

    void start() override;
    void stop() override;
    bool available() override;
    std::shared_ptr<DepthFrame> getFrame() override;
    void save(const std::string& path);

private:
    libfreenect2::Freenect2 freenect2_;
    std::unique_ptr<libfreenect2::Freenect2Device> dev_;
    std::unique_ptr<libfreenect2::PacketPipeline> pipeline_;

    bool frame_available_;
    std::shared_ptr<DepthFrame> frame_;

    std::thread freenect_thread_;
    std::condition_variable available_cv_;
    std::mutex available_mx_;
    std::mutex frame_mx_;
};


#endif //CPPND_CAPSTONE_KINECTDEPTHCAPTURE_H
