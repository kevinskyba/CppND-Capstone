//
// Created by kevinskyba on 05.01.20.
//

#include "FileDepthCapture.h"
#include "FrameUtils.h"
#include <utility>
#include <iostream>

FileDepthCapture::FileDepthCapture(std::string folder) {
    this->folder_ = std::move(folder);
}

FileDepthCapture::~FileDepthCapture() {

}

void FileDepthCapture::start() {
    current_frame_ = 0;
    frames_ = FrameUtils::LoadFramesFromFolder(this->folder_);
}

bool FileDepthCapture::available() {
    return current_frame_ < frames_.size();
}

std::shared_ptr<DepthFrame> FileDepthCapture::getFrame() {
    return frames_[current_frame_++];
}

void FileDepthCapture::stop() {

}
