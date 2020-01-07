//
// Created by kevinskyba on 01.01.20.
//

#ifndef CPPND_CAPSTONE_DEPTHFRAME_H
#define CPPND_CAPSTONE_DEPTHFRAME_H


#include <cstddef>
#include <bits/unique_ptr.h>

/**
 * DepthFrame is a container for depth sensor data. The data of DepthFrame should be fixed by perspective, so that
 * there is no stretching from perspective anymore.
 */
class DepthFrame {
public:
    const size_t width;
    const size_t height;

    DepthFrame(size_t width, size_t height, float* data);

    float* getData() const;

private:
    std::unique_ptr<float> data_;
};


#endif //CPPND_CAPSTONE_DEPTHFRAME_H
