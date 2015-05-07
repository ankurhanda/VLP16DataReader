/* Author: Ankur Handa
  Date  : 7th May 2015
  Place : Cambridge
 */


#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include "./src/HDLGrabber/custom_hdl_grabber.h"

//#define HAVE_PCAP 1

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef typename Cloud::ConstPtr CloudConstPtr;

boost::mutex cloud_mutex_;
CloudConstPtr cloud_;

void cloud_callback (const CloudConstPtr& cloud)
{
  boost::mutex::scoped_lock lock (cloud_mutex_);
  cloud_ = cloud;
}

int main(void)
{
//    CustomHDLGrabber (const boost::asio::ip::address& ipAddress,
//                      const unsigned short port,
//                      const std::string& correctionsFile = "");


    boost::asio::ip::address ip;
    boost::asio::ip::address curIPAddress;
    curIPAddress = ip.from_string("192.168.3.255");

    std::cout<<"IP address = " << curIPAddress << std::endl;


    pcl::CustomHDLGrabber hdl_grabber(curIPAddress,
                                      2368,
                                      "../data/VLP16_db.xml");

//    pcl::CustomHDLGrabber hdl_grabber("../data/VLP16_db.xml",
//                                      "../data/VLP16_test_data.pcap");

    boost::function<void (const CloudConstPtr&)> cloud_cb = boost::bind (
                &cloud_callback, _1);

    boost::signals2::connection cloud_connection = hdl_grabber.registerCallback (
                cloud_cb);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    hdl_grabber.start();

    bool updated = false;

    while(true)
    {
        CloudConstPtr cloud;

        if (cloud_mutex_.try_lock())
        {
            cloud_.swap (cloud);
            cloud_mutex_.unlock ();
        }

        if ( cloud )
        {

            if( !updated )
            {
                viewer->addPointCloud<pcl::PointXYZ> (cloud);
                updated = true;
            }
            else
            {
                viewer->updatePointCloud<pcl::PointXYZ> (cloud);
            }

            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);

        }

        viewer->spinOnce(10);
    }

    hdl_grabber.stop();

    cloud_connection.disconnect();

}
