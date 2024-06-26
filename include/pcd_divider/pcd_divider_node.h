#ifndef PCD_DIVIDER_NODE_H
#define PCD_DIVIDER_NODE_H

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>

class PCDDivider
{
public:
    PCDDivider();
    void run();

private:
    ros::NodeHandle nh_;
    std::string input_pcd_file_;
    std::string output_pcd_file_filtered_;
    std::string output_pcd_file_down_;
    std::string output_pcd_file_top_;
    float z_threshold_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCDFile(const std::string& filename);
    pcl::PointCloud<pcl::PointXYZ>::Ptr applyStatisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void dividePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_half1, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_half2);
    void savePCDFile(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};

#endif // PCD_DIVIDER_NODE_H
