#include <iostream>
#include <string>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <ros/package.h>
#include <ros/ros.h>

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

using namespace std;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pcd_merge_node");
    ros::NodeHandle nh("~");

    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;
    double filter_leaf_size_;
    std::string merge_pcd_file_name_;

    if (!nh.param("min_x", min_x_, -50.0))
    {
        ROS_WARN("No param min_x");
    }
    nh.param("max_x", max_x_, 50.0);
    nh.param("min_y", min_y_, -50.0);
    nh.param("max_y", max_y_, 50.0);
    nh.param("min_z", min_z_, 0.0);
    nh.param("max_z", max_z_, 50.0);
    nh.param("filter_leaf_size", filter_leaf_size_, 0.1);
    nh.param("merged_pcd_name", merge_pcd_file_name_, std::string("merged.pcd"));

    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::VoxelGrid<pcl::PointXYZI> filter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_add(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> filter;

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;

    DIR *pDir;
    struct dirent *ptr;

    std::string package_name = "pcd_manage";
    std::string package_path = ros::package::getPath(package_name);
    std::string pcd_files_path = package_path + string("/pcd_raw/");
    std::string pcd_save_addr = package_path + string("/pcd_merged/");
    std::string merge_pcd_file_absaddr_ = pcd_save_addr + merge_pcd_file_name_;

    if (!(pDir = opendir(pcd_files_path.c_str())))
    {
        std::cout << "Folder doesn't Exist!" << std::endl;
        return -1;
    }

    while ((ptr = readdir(pDir)) != nullptr)
    {
        // 这部分代码的作用是在遍历目录时跳过当前目录和上级目录，以避免对它们进行处理。在UNIX-like系统中，每个目录都包含两个特殊的目录项：
        // 当前目录（.）：表示目录本身。
        // 上级目录（..）：表示目录的父目录。
        if (strcmp(ptr->d_name, ".") == 0 && strcmp(ptr->d_name, "..") == 0)
            continue;

        std::string filename = ptr->d_name;
        if (filename.length() >= 4 && filename.substr(filename.length() - 4) == ".pcd")
        {
            pcl::io::loadPCDFile(pcd_files_path + string(ptr->d_name), *cloud);

            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(min_x_, max_x_);
            pass.filter(*filteredCloud);

            pass.setInputCloud(filteredCloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(min_y_, max_y_);
            pass.filter(*filteredCloud);

            pass.setFilterFieldName("z");
            pass.setFilterLimits(min_z_, max_z_);
            pass.filter(*filteredCloud);

            if (filter_leaf_size_ > 0)
            {
                filter.setInputCloud(filteredCloud);
                filter.setLeafSize(filter_leaf_size_, filter_leaf_size_, filter_leaf_size_);
                filter.filter(*filteredCloud);
            }

            *cloud_add = *cloud_add + *filteredCloud;
            cloud->clear();
            filteredCloud->clear();
            std::cout << pcd_files_path + ptr->d_name << endl;
        }
    }
    closedir(pDir);

    pcl::io::savePCDFileBinary(merge_pcd_file_absaddr_, *cloud_add);

    ROS_WARN("finish pcd merge!");
    return 0;
}
