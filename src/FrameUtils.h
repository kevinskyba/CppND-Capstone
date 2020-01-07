//
// Created by kevinskyba on 05.01.20.
//

#ifndef CPPND_CAPSTONE_FRAMEUTILS_H
#define CPPND_CAPSTONE_FRAMEUTILS_H

#include <boost/shared_ptr.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <fstream>
#include "DepthFrame.h"

namespace FrameUtils {

    /**
     * This method loads all DepthFrames from .frame files in the given folder. The custom written .file format is
     * described in KinectDepthCapture.
     */
    static std::vector<std::shared_ptr<DepthFrame>> LoadFramesFromFolder(const std::string& folder) {

        std::vector<std::string> paths = {};
        for (auto& entry : std::filesystem::recursive_directory_iterator(folder)) {
            if (entry.path().extension().generic_string() == ".frame") {
                paths.push_back(entry.path());
            }
        }
        std::sort(paths.begin(), paths.end(), [](const std::string& a, const std::string& b) {
            return a < b;
        });

        std::vector<std::shared_ptr<DepthFrame>> frames_ = {};
        for (auto path : paths) {
            std::ifstream fin;
            fin.open(path, std::ios::binary | std::ios::in);

            size_t width = 0, height = 0;
            fin.read(reinterpret_cast<char *>(&width), sizeof(size_t));
            fin.read(reinterpret_cast<char *>(&height), sizeof(size_t));

            int size = width * height;
            auto* data = new float[size];
            int i = 0;

            while(fin) {
                fin.read(reinterpret_cast<char *>(&data[i++]), sizeof(float));
            }
            fin.close();

            auto frame = std::make_shared<DepthFrame>(width, height, data);
            frames_.push_back(std::move(frame));

            std::cout << "Added file " << path << " to frames" << std::endl;
        }

        return frames_;
    }

    /**
     * This will add a DepthFrame to a point cloud and remove all perspective, so that the points in the point cloud
     * are actualy world coordinates.
     */
    static void AddFrameToPLC(const DepthFrame *frame, const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud) {
        cloud->clear();
        cloud->is_dense = true;

        for (int yi = 0; yi < frame->height; yi++) {
            for (int xi = 0; xi < frame->width; xi++) {
                const auto depth = frame->getData()[yi * frame->width + xi] / 5.0f;

                const float xzFactor = 0.708f * 2.f;
                const float yzFactor = 0.577350269f * 2.f;
                const float resolutionX = frame->width;
                const float resolutionY = frame->height;

                if (!std::isnan(depth) && std::abs(depth) >= 0.0001) {
                    pcl::PointXYZRGB point;

                    // Retrieve Mapped Coordinates
                    auto x = static_cast<float>(xi);
                    auto y = static_cast<float>(yi);
                    float z = depth;

                    // convertDepthToWorldCoordinates as discussed here:
                    // https://stackoverflow.com/questions/39389279/c-kinect-v2-freenect2-how-to-convert-depth-data-to-real-world-coordinates
                    float normalizedX = x / resolutionX - .5f;
                    float normalizedY = .5f - y / resolutionY;

                    point.x = (normalizedX * z * xzFactor);
                    point.y = (normalizedY * z * yzFactor);
                    point.z = z;

                    // White color
                    point.r = 255;
                    point.g = 255;
                    point.b = 255;

                    // Set Point to Point Cloud
                    cloud->push_back(point);
                }
            }
        }
    }
};


#endif //CPPND_CAPSTONE_FRAMEUTILS_H
