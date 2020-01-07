
#include <iostream>
#include <boost/move/move.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "cxxopts.h"
#include "KinectDepthCapture.h"
#include "FrameUtils.h"

/**
 * This function is returning a lambda function. This has to be like this because PCL is not able to take more than one
 * parameter in the keyboardEvent function. Using this technique of a "function generator" we can artifically add more
 * parameters to this function.
 * @param outFolder The folder to save images to
 */
std::function<void(const pcl::visualization::KeyboardEvent&)> keyboardEventOccurred(const std::string& outFolder, KinectDepthCapture& kinectDepthCapture)
{
    return [=, &kinectDepthCapture, &outFolder](const pcl::visualization::KeyboardEvent &event){
        if (event.getKeySym() == "t" && event.keyDown())
            kinectDepthCapture.save(outFolder);
    };
}

int main(int argc, char* argv[]) {

    cxxopts::Options options(argv[0], "");
    std::string outFolder;
    try {
        options.add_options()
                ("o,out", "Output folder", cxxopts::value<std::string>()->default_value("./"));
        cxxopts::ParseResult argResult = options.parse(argc, argv);
        outFolder = argResult["out"].as<std::string>();
    } catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    KinectDepthCapture capture;

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Fix wrong sensor orientation of kinect
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitY()));
    cloud->sensor_orientation_ = transform.rotation();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("CppND-Capstone");

    // Register keyboard callback
    typedef void callback (const pcl::visualization::KeyboardEvent&);
    viewer->registerKeyboardCallback(keyboardEventOccurred(outFolder, capture));

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
        FrameUtils::AddFrameToPLC(frame.get(), cloud);

        if(!viewer->updatePointCloud(cloud, "cloud")) {
            viewer->addPointCloud(cloud, "cloud");
            viewer->addText("Press 't' to capture a frame", 25, 25);
        }

        viewer->spinOnce();
    }

}