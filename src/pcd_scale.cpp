/**
 * @file pcd_scale.cpp
 * @author ninedayhx (1170535490@qq.com)
 * @brief
 * @version 0.1
 * @date 2023-05-05
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <iostream>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <eigen3/Eigen/Eigen>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        cout << "usage: pcd_scale [input.pcd] [scale]" << endl;
        cout << "scale should be a float number" << endl;
    }

    string file_path = argv[1];
    float scale = atof(argv[2]);

    PointCloud::Ptr pCloud(new PointCloud);
    PointCloud::Ptr pCloud_scale(new PointCloud);

    if (pcl::io::loadPCDFile(argv[1], *pCloud) < 0)
    {
        cout << "error: load pcd file failed" << endl;
        return -1;
    }

    pcl::PointXYZ min_pt, max_pt;

    // 调用 getMinMax3D 函数
    pcl::getMinMax3D(*pCloud, min_pt, max_pt);

    // 计算长、宽、高
    float length = max_pt.x - min_pt.x;
    float width = max_pt.y - min_pt.y;
    float height = max_pt.z - min_pt.z;

    // 输出结果
    std::cout << "Length: " << length << std::endl;
    std::cout << "Width: " << width << std::endl;
    std::cout << "Height: " << height << std::endl;

    cout << "solution1: 遍历" << endl;
    for (auto &point : pCloud->points)
    {
        point.x *= scale;
        point.y *= scale;
        point.z *= scale;
        pCloud_scale->push_back(point);
    }

    cout << "solution2: 变换" << endl;
    // Eigen::Matrix4f trans = Eigen::Matrix4f::Identity();

    // trans(0, 0) = scale;
    // trans(1, 1) = scale;
    // trans(2, 2) = scale;

    // pcl::transformPointCloud(*pCloud, *pCloud_scale, trans);

    pcl::getMinMax3D(*pCloud_scale, min_pt, max_pt);

    // 计算长、宽、高
    length = max_pt.x - min_pt.x;
    width = max_pt.y - min_pt.y;
    height = max_pt.z - min_pt.z;

    // 输出结果
    std::cout << "scale Length: " << length << std::endl;
    std::cout << "scale Width: " << width << std::endl;
    std::cout << "scale Height: " << height << std::endl;

    if (pcl::io::savePCDFile("output.pcd", *pCloud))
    {
        cout << "error: save pcd file failed" << endl;
        return -1;
    }

    return 0;
}
