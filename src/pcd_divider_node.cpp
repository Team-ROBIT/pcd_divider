#include "../include/pcd_divider/pcd_divider_node.h"

// OctoMap headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>

// ROS headers
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// PCL headers
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

PCDDivider::PCDDivider() : nh_("~")
{
    nh_.param("pcd_file", input_pcd_file_, std::string("input.pcd"));
    nh_.param("output_pcd_file_filtered", output_pcd_file_filtered_, std::string("/home/sh/catkin_ws/pcd/map_filtered.pcd"));
    nh_.param("output_pcd_file_down", output_pcd_file_down_, std::string("/home/sh/catkin_ws/pcd/map_down.pcd"));
    nh_.param("output_pcd_file_top", output_pcd_file_top_, std::string("/home/sh/catkin_ws/pcd/map_top.pcd"));
    nh_.param("output_octomap_file", output_octomap_file_, std::string("/home/sh/catkin_ws/maps/map_3d.ot"));
    nh_.param("z_threshold", z_threshold_, 0.0f);
}

void PCDDivider::run()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadPCDFile(input_pcd_file_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = applyStatisticalOutlierRemoval(cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_half1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_half2(new pcl::PointCloud<pcl::PointXYZ>);
    dividePointCloud(cloud_filtered, cloud_half1, cloud_half2);

    savePCDFile(output_pcd_file_filtered_, cloud_filtered);
    savePCDFile(output_pcd_file_down_, cloud_half1);
    savePCDFile(output_pcd_file_top_, cloud_half2);
    saveOctomapFile(output_octomap_file_, cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDDivider::loadPCDFile(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the file %s\n", filename.c_str());
        return nullptr;
    }
    std::cout << "Loaded " << cloud->width * cloud->height << " data points from " << filename << std::endl;
    return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDDivider::applyStatisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    return cloud_filtered;
}

void PCDDivider::dividePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_half1, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_half2)
{
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        if (cloud->points[i].z < z_threshold_)
        {
            inliers->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_half1);

    extract.setNegative(true);
    extract.filter(*cloud_half2);
}

void PCDDivider::savePCDFile(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    if (pcl::io::savePCDFileASCII(filename, *cloud) == -1)
    {
        PCL_ERROR("Couldn't write the file %s\n", filename.c_str());
    }
    else
    {
        std::cout << "Saved " << filename << " with " << cloud->width * cloud->height << " data points." << std::endl;
    }
}

void PCDDivider::saveOctomapFile(const std::string &filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    // OctoMap 객체 생성
    octomap::OcTree tree(0.05);

    // 포인트 클라우드 데이터를 OctoMap에 복사
    for (auto p : cloud->points)
    {
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }

    tree.updateInnerOccupancy();

    if (tree.write(filename))
    {
        ROS_INFO("Octomap saved as %s", filename.c_str());
    }
    else
    {
        ROS_ERROR("Failed to write octomap to file %s", filename.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_divider_node");

    PCDDivider pcd_divider;
    pcd_divider.run();

    return 0;
}
