
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/io.h>
#include <pcl/registration/super4pcs.h>
#include <super4pcs/shared4pcs.h>
#include "FileDepthCapture.h"
#include "cxxopts.h"
#include "FrameUtils.h"

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
 * This method removes a percentage of the points in the given point cloud.
 */
void ScaleCloud(float leafSize, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>& cloud) {
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setLeafSize(leafSize, leafSize, leafSize);
    sor.filter(*cloud);
}

int main(int argc, char* argv[]) {

    // Parse program args
    cxxopts::Options options(argv[0], "");
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

    try {
        capture.start();
    } catch(const std::exception &e) {
        std::cout << "Error while starting: " << e.what() << std::endl;
        return -1;
    }

    // Gather all frames from file capture
    std::vector<std::shared_ptr<DepthFrame>> frames;
    while(capture.available()) {
        frames.push_back(std::move(capture.getFrame()));
    }

    // Create the "target" initially. The target is the current recreated point cloud
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> target = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    FrameUtils::AddFrameToPLC(frames[0].get(), target); // target starts with the first frame
    CropCloud(-25, -80, 0, 50, 100, 300, target); // Need to keep the important stuff in focus
    ScaleCloud(0.0005f, target);

    // Make sure to remove the sensor orientation of kinect
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI /*180°*/, Eigen::Vector3f::UnitY()));
    target->sensor_orientation_ = transform.rotation();

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = boost::make_shared<pcl::visualization::PCLVisualizer>("CppND-Capstone");
    viewer->addPointCloud(target, "cloud");
    viewer->spinOnce();

    // Super4PCS is a third party algorithm able to stitch together point clouds from multiple angles into one single
    // pointcloud.
    pcl::Super4PCS<pcl::PointXYZRGB, pcl::PointXYZRGB> super4pcs;
    super4pcs.options_ = GlobalRegistration::Match4PCSOptions {  };
    super4pcs.options_.max_time_seconds = 30;
    super4pcs.options_.delta = 1.f;
    super4pcs.options_.configureOverlap(0.2f);

    for (int i = 1; i < frames.size(); i++) {
        // "a" is the current space. It is always set to the current target, which is the most up-to-date
        // reconstructed scene.
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> a = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::copyPointCloud(*target, *a);

        // "b" is the next frame to be added to "target".
        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> b = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        FrameUtils::AddFrameToPLC(frames[i].get(), b);
        CropCloud(-25, -80, 0, 50, 100, 300, b);
        ScaleCloud(0.0005f, b);

        super4pcs.setInputSource(b);
        super4pcs.setInputTarget(a);

        pcl::PointCloud<pcl::PointXYZRGB> res;
        super4pcs.align(res);
        *target += res; // Add the aligned point cloud of "b" to target

        if (super4pcs.hasConverged()) {
            std::cout << "Matched frame " << i << " by " << super4pcs.getFitnessScore() << std::endl;
            viewer->updatePointCloud(target, "cloud");
            viewer->spinOnce();
        } else {
            std::cout << "Could not match frame " << i << std::endl;
            return 1;
        }
    }

    while(true) {
        viewer->spinOnce();
    }

}
