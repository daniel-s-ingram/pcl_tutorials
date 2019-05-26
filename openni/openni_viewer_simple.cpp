#include <memory>
#include <pcl/io/openni_grabber.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "Eigen/Core"

class SimpleOpenNIViewer
{
    public:
        SimpleOpenNIViewer() : viewer("PCL OpenNI Viewer")
        {
            T = Eigen::Affine3f::Identity();
            T.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
            transformedCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
        }

        void cloud_cb_(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr& cloud)
        {
            if (!viewer.wasStopped())
            {
                pcl::transformPointCloud(*cloud, *transformedCloud, T);
                viewer.showCloud(transformedCloud);
            }
        }

        void run()
        {
            std::unique_ptr<pcl::Grabber> interface(new pcl::OpenNIGrabber());
            boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)>
                f = boost::bind(&SimpleOpenNIViewer::cloud_cb_, this, _1);
            interface->registerCallback(f);
            interface->start();
            while(!viewer.wasStopped())
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            interface->stop();
        }

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transformedCloud;
        pcl::visualization::CloudViewer viewer;
        Eigen::Affine3f T;
};

int main()
{
    SimpleOpenNIViewer v;
    v.run();
    return 0;
}