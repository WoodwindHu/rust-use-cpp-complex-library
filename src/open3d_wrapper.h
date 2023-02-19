//
// Created by Administrator on 2023/2/3.
//


#pragma once




#include "open3d_wrapper.h"

#include "open3d/Open3D.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
void visualize_pc(const std::string & filename);

std::shared_ptr<open3d::geometry::PointCloud> read_pcd(const std::string & filename) {
    open3d::PrintOpen3DVersion();
    // load point cloud from kitti-format point cloud file
    std::ifstream input(filename, std::ios::binary);

    if (!input.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return nullptr;
    }
    input.seekg(0, std::ios::end); // 定位到文件末尾
    std::streampos file_size = input.tellg(); // 获取文件大小
    input.seekg(0, std::ios::beg); // 定位到文件开头
    std::vector<float> data(file_size/sizeof(float), 0);
    input.read(reinterpret_cast<char*>(data.data()), file_size);
    input.close();

    // create point cloud
    auto pcd = std::make_shared<open3d::geometry::PointCloud>();
    pcd->points_.resize(data.size() / 4);
    pcd->colors_.resize(data.size() / 4);
    for (size_t i = 0; i < data.size() / 4; i++)
    {
        float x = data[i * 4];
        float y = data[i * 4 + 1];
        float z = data[i * 4 + 2];
        float intensity = data[i * 4 + 3];

        pcd->points_[i] = Eigen::Vector3d(x, y, z);
        pcd->colors_[i] = Eigen::Vector3d(0, 0, intensity);
    }

    return pcd;
}

void visualize(std::shared_ptr<open3d::geometry::PointCloud> pcd) {
    // visualize point cloud
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow("Open3D Point Cloud Visualization", 1600, 900);
    visualizer.AddGeometry(pcd);
    visualizer.GetRenderOption().point_size_ = 1.0;
    visualizer.ResetViewPoint(true);
    visualizer.Run();
}

void visualize_pc(const std::string & filename) {
    auto pcd = read_pcd(filename);
    visualize(pcd);
}