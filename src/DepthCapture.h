//
// Created by kevinskyba on 01.01.20.
//

#ifndef CPPND_CAPSTONE_DEPTHCAPTURE_H
#define CPPND_CAPSTONE_DEPTHCAPTURE_H


#include <memory>
#include "DepthFrame.h"

/**
 * DepthCapture is a generic abstract class which can be overriden to provide different sources of DepthFrames.
 */
class DepthCapture {
public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual bool available() = 0;
    virtual std::shared_ptr<DepthFrame> getFrame() = 0;
};


#endif //CPPND_CAPSTONE_DEPTHCAPTURE_H
