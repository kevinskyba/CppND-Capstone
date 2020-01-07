
#include <iostream>
#include <boost/move/move.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "FrameUtils.h"
#include "KinectDepthCapture.h"

int main(int argc, char* argv[]) {

    KinectDepthCapture capture;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Fix wrong sensor orientation of kinect
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
        FrameUtils::AddFrameToPLC(frame.get(), cloud);

        if(!viewer->updatePointCloud(cloud, "cloud")) {
            viewer->addPointCloud(cloud, "cloud");
        }

        viewer->spinOnce();
    }

}
