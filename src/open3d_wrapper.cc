//
// Created by Administrator on 2023/2/3.
//


#include "open3d/Open3D.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include "open3d_wrapper.h"
// #include "open3d_cxx_wrapper.h"
#include "cxxgen.h"
// #include "open3d-rs/src/binds.rs.h"

std::shared_ptr<open3d::geometry::PointCloud> construct_pointcloud(const rust::Vec<float> & data) {
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

std::unique_ptr<open3d::visualization::Visualizer> get_visualizer(const std::string & title, int width, int height) {
    auto visualizer = std::make_unique<open3d::visualization::Visualizer>();
    visualizer->CreateVisualizerWindow(title, width, height);
    visualizer->GetRenderOption().point_size_ = 1.0;
    return visualizer;
}

std::shared_ptr<open3d::geometry::LineSet> generate_boundingboxes_line_set(const Vertexes vertexes) {
    auto lineset = std::make_shared<open3d::geometry::LineSet>();
    lineset->points_.resize(24);
    for (size_t i = 0; i < 24; i++)
    {
        float x = vertexes.data[i * 3];
        float y = vertexes.data[i * 3 + 1];
        float z = vertexes.data[i * 3 + 2];

        lineset->points_[i] = Eigen::Vector3d(x, y, z);
    }

    lineset->lines_.resize(12);
    lineset->lines_[0] = Eigen::Vector2i(0, 1);
    lineset->lines_[1] = Eigen::Vector2i(1, 3);
    lineset->lines_[2] = Eigen::Vector2i(2, 3);
    lineset->lines_[3] = Eigen::Vector2i(2, 0);
    lineset->lines_[4] = Eigen::Vector2i(4, 5);
    lineset->lines_[5] = Eigen::Vector2i(5, 7);
    lineset->lines_[6] = Eigen::Vector2i(6, 7);
    lineset->lines_[7] = Eigen::Vector2i(6, 4);
    lineset->lines_[8] = Eigen::Vector2i(0, 4);
    lineset->lines_[9] = Eigen::Vector2i(1, 5);
    lineset->lines_[10] = Eigen::Vector2i(2, 6);
    lineset->lines_[11] = Eigen::Vector2i(3, 7);

    return lineset;
}

std::unique_ptr<open3d::visualization::Visualizer> add_lineset(std::unique_ptr<open3d::visualization::Visualizer> visualizer, std::shared_ptr<open3d::geometry::LineSet> lineset) {
    lineset->PaintUniformColor(Eigen::Vector3d(0.0, 1.0, 0.0));
    visualizer->AddGeometry(lineset);
    return visualizer;
}

std::unique_ptr<open3d::visualization::Visualizer> add_geometry(std::unique_ptr<open3d::visualization::Visualizer> visualizer, std::shared_ptr<open3d::geometry::PointCloud> pcd) {
    visualizer->AddGeometry(pcd);
    return visualizer;
}

void run_visualizer(std::unique_ptr<open3d::visualization::Visualizer> visualizer) {
    visualizer->Run();
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

void test_rust_slice() {
    rust::Vec<float> data = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    test_slice(data);
}