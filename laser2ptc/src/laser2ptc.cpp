#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

using namespace pcl;

ros::Publisher ptc_pub;
laser_geometry::LaserProjection projector;

void s2ptc_callback(const sensor_msgs::LaserScanConstPtr& laser_readings)
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header.frame_id = laser_readings->header.frame_id;
    cloud.header.stamp    = laser_readings->header.stamp;
    projector.projectLaser(*laser_readings, cloud);
    ptc_pub.publish(cloud);
    ROS_INFO("Quantos pontos? %zu", cloud.data.size());

    pcl::PointCloud<PointXYZ>::Ptr pclcloud (new PointCloud<PointXYZ>());

    pcl::fromROSMsg(cloud, *pclcloud);

    pcl::io::savePLYFileASCII("/home/grin/Desktop/frame_laser.ply", *pclcloud);

    ROS_WARN("Veja se salvou na area de trabalho!");
    ros::shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser2ptc");
    ros::NodeHandle nh;

    ros::Subscriber laser_sub = nh.subscribe("/scan", 1000, s2ptc_callback);
    ptc_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan2/PointCloud", 1000);

    ros::spin();

    return 0;
}
