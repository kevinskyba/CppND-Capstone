//
// Created by kevinskyba on 05.01.20.
//

#ifndef CPPND_CAPSTONE_FILEDEPTHCAPTURE_H
#define CPPND_CAPSTONE_FILEDEPTHCAPTURE_H

#include <memory>
#include <vector>
#include "DepthCapture.h"

/**
 * FileDepthCapture is the implementation of DepthCapture which does not require any real device, but instead reads
 * pre-recorded frames from a folder and plays them just as if they came from a real device.
 */
class FileDepthCapture : public DepthCapture {
public:

    FileDepthCapture(std::string folder);
    ~FileDepthCapture();

    void start() override;
    void stop() override;
    bool available() override;
    std::shared_ptr<DepthFrame> getFrame() override;

private:
    std::string folder_;
    std::vector<std::shared_ptr<DepthFrame>> frames_;
    int current_frame_;
};

#endif //CPPND_CAPSTONE_FILEDEPTHCAPTURE_H
