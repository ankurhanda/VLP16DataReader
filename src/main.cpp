/* Author: Ankur Handa
  Date  : 7th May 2015
  Place : Cambridge
 */


#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <cvd/image.h>
#include <cvd/rgb.h>
#include <cvd/image_ref.h>
#include <cvd/image_io.h>
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


    int count = 0;

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

            int height = cloud->height;
            int width  = cloud->width;

            double pix_res_x = 0.4;

            int widthImg = (int)(360.0/pix_res_x);

            CVD::Image<u_int16_t>LiDARImage(CVD::ImageRef(widthImg,16));

            for(int yy = 0; yy < 16; yy++)
            {
                for(int xx = 0; xx < widthImg; xx++)
                {
                    LiDARImage[CVD::ImageRef(xx,yy)] = 0;
                }
            }
//            std::cout<<"height = " << height <<", width = " << width << std::endl;

            double dist_max = -1E20;

            std::vector<float>yaws;
            std::vector<float>pitches;

            char pfileName[100];

            sprintf(pfileName,"points_%06d.txt",count);

            ofstream pointfile(pfileName);

            for(int xx = 0; xx < width*height; xx++)
            {
                pcl::PointXYZ p = cloud->at(xx);

                pointfile<<p.x<<", "<<p.y<<", "<<p.z<< std::endl;

                double xr = p.x;
                double yr = p.y;
                double zr = p.z;

                double xrs = xr*xr;
                double yrs = yr*yr;
                double zrs = zr*zr;

                double dist = sqrt(xrs + yrs + zrs);

                double yawRAD   = atan2(yr,xr) * 180.0f / M_PI; // atan2 returns in [-pi..+pi]
                double pitchRAD = atan2(-zr,sqrt(xrs+yrs)) * 180.0f / M_PI; // returns in [-pi..+pi]

//                std::cout<<"yawRAD = " << yawRAD <<", pitchRAD = " << pitchRAD << std::endl;

//                printf(" yaw = %f, pitch = %f ", yawRAD, pitchRAD);

                yaws.push_back(yawRAD);
                pitches.push_back(pitchRAD);

                int hi = (int)(round((yawRAD + 180.0f)/pix_res_x));

                int vi = (int)(round((pitchRAD + 15.0)/2));

                if ( dist_max < dist )
                    dist_max = dist;

//                if ( pitchRAD && yawRAD )
                    LiDARImage[CVD::ImageRef(hi,vi)] = (u_int16_t)(dist*5000.0);

            }

            pointfile.close();

            if ( dist_max > 10 )
                std::cout<<"count = " << count <<" dist_max = " << dist_max << std::endl;

            static char fileName[300];

            sprintf(fileName,"LiDARImage%06d.png",count);

            CVD::img_save(LiDARImage,fileName);

            static char ofileName[300];

            sprintf(ofileName,"yaws_pitches_%06d.txt",count);

            ofstream outfile(ofileName);

            for(int i = 0; i < yaws.size(); i++)
            {
                outfile << yaws.at(i)<<", "<<pitches.at(i)<< std::endl;
            }

            outfile.close();

            count++;
        }

        viewer->spinOnce(10);
    }

    hdl_grabber.stop();

    cloud_connection.disconnect();

}
