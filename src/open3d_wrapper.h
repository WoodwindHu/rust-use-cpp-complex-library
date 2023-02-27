//
// Created by Administrator on 2023/2/3.
//


#pragma once

#include <string>
#include "open3d/Open3D.h"


struct Vertexes {
    float data[24];
};

void visualize_pc(const std::string & filename);
std::shared_ptr<open3d::geometry::PointCloud> read_pcd(const std::string & filename);
void visualize(std::shared_ptr<open3d::geometry::PointCloud> pcd);
void test_rust_slice();
std::unique_ptr<open3d::visualization::Visualizer> get_visualizer(const std::string & title, int width, int height);
std::unique_ptr<open3d::visualization::Visualizer> add_geometry(std::unique_ptr<open3d::visualization::Visualizer> visualizer, std::shared_ptr<open3d::geometry::PointCloud> pcd);
void run_visualizer(std::unique_ptr<open3d::visualization::Visualizer> visualizer);
std::shared_ptr<open3d::geometry::LineSet> generate_boundingboxes_line_set(Vertexes vertexes);
std::unique_ptr<open3d::visualization::Visualizer> add_lineset(std::unique_ptr<open3d::visualization::Visualizer> visualizer, std::shared_ptr<open3d::geometry::LineSet> lineset);