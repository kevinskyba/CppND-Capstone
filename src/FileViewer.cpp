
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_box.h>
#include "FrameUtils.h"
#include "FileDepthCapture.h"
#include "cxxopts.h"

/**
 * This method takes a cloud and cuts away all pixels outside defined by the box defined by the min- and max parameters.
 */
void CropCloud(int minX, int minY, int minZ, int maxX, int maxY, int maxZ, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud) {
    pcl::CropBox<pcl::PointXYZRGB> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    boxFilter.setInputCloud(cloud);
    boxFilter.filter(*cloud);
}

/**
 * This function is returning a lambda function. This has to be like this because PCL is not able to take more than one
 * parameter in the keyboardEvent function. Using this technique of a "function generator" we can artifically add more
 * parameters to this function.
 */
std::function<void(const pcl::visualization::KeyboardEvent&)> keyboardEventOccurred(FileDepthCapture& fileDepthCapture, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud)
{
    return [=, &fileDepthCapture, &cloud](const pcl::visualization::KeyboardEvent &event){
        if (event.getKeySym() == "p" && event.keyDown()) {
            if (!fileDepthCapture.available()) {
                return;
            }

            auto frame = fileDepthCapture.getFrame();
            FrameUtils::AddFrameToPLC(frame.get(), cloud);
            CropCloud(-25, -80, 0, 50, 100, 300, cloud);
        }
    };
}

int main(int argc, char* argv[]) {

    cxxopts::Options options(argv[0], " - example cli options");
    std::string folder;
    try {
        options.add_options()
                ("i,in", "Frame folder", cxxopts::value<std::string>()->default_value("./"));
        cxxopts::ParseResult argResult = options.parse(argc, argv);
        folder = argResult["in"].as<std::string>();
    } catch (const cxxopts::OptionException& e)
    {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    FileDepthCapture capture(folder);

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    // Fix wrong sensor orientation of kinect
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitY()));
    cloud->sensor_orientation_ = transform.rotation();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("CppND-Capstone");

    // Register keyboard callback
    typedef void callback (FileDepthCapture&, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>&);
    viewer->registerKeyboardCallback(keyboardEventOccurred(capture, cloud));

    try {
        capture.start();
    } catch(const std::exception &e) {
        std::cout << "Error while starting: " << e.what() << std::endl;
        return -1;
    }

    std::shared_ptr<DepthFrame> frame;
    while(true) {
        if(!viewer->updatePointCloud(cloud, "cloud")) {
            viewer->addPointCloud(cloud, "cloud");
            viewer->addText("Press 'p' to go to the next frame", 25, 25);
        }

        viewer->spinOnce();
    }

}
