//
// Created by kevinskyba on 01.01.20.
//

#include "DepthFrame.h"

DepthFrame::DepthFrame(size_t width, size_t height, float *data)
    : width(width), height(height), data_(data) {

}

float *DepthFrame::getData() const {
    return data_.get();
}
