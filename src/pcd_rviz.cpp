/*
 * @Author: Xiaoxun Zhang
 * @Date: 2025-08-04 19:43:59
 * @LastEditTime: 2025-08-04 22:09:43
 * @Description:
 */

#include <iostream>
#include <string>

#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "pcl_rviz_node");
    ros::NodeHandle nh;

    string pcd_file = argv[1];

    sensor_msgs::PointCloud2 cloud_msg;
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/full_cloud", 10);

    // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new
    // pcl::PointCloud<pcl::PointXYZINormal>()); pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new
    // pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    if (pcl::io::loadPCDFile(pcd_file, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file cave_full_pcd.pcd ^.^\n");
        return (-1);
    }

    pcl::toROSMsg(*cloud, cloud_msg);
    ros::Duration(2).sleep();
    while (ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();
        cloud_msg.header.frame_id = "map";
        cloud_pub.publish(cloud_msg);

        ros::Duration(10).sleep();

        ros::spinOnce();
    }

    return 0;
}