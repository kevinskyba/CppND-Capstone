
#include <iostream>
#include <boost/move/move.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "KinectDepthCapture.h"

int main() {

    KinectDepthCapture capture;


    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitY()));
    cloud->sensor_orientation_ = transform.rotation();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("CppND-Capstone");

    try {
        capture.start();
    } catch(const std::exception &e) {
        std::cout << "Error while starting: " << e.what() << std::endl;
        return -1;
    }

    std::shared_ptr<DepthFrame> frame;
    while(true) {
        if (!capture.available()) {
            return 0;
        }

        frame = capture.getFrame();

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

        if(!viewer->updatePointCloud(cloud, "cloud")) {
            viewer->addPointCloud(cloud, "cloud");
        }

        viewer->spinOnce();
    }

}
