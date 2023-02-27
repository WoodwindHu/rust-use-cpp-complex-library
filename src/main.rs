mod binds;
// mod pointcloud_reader;
mod calibration;
mod annotation;
use anyhow::Result;

use clap::Parser;

use crate::{calibration::Calibration, annotation::{read_annotation_file, get_vertexes_from_kitti_annos}};

/// Simple program to visualize point cloud
#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
   /// Directory of the kitti-format dataset
   #[arg(short, long)]
   dir: String,

   /// Index of the point cloud to visualize
   #[arg(short, long)]
   index: String,
}

fn main() {
    let args = Args::parse();
    let bin_filename = format!("{}/velodyne/{}.bin", args.dir, args.index);
    let calib_filename = format!("{}/calib/{}.txt", args.dir, args.index);
    let label_filename = format!("{}/label_2/{}.txt", args.dir, args.index);
    let image_filename = format!("{}/image_2/{}.png", args.dir, args.index);
    cxx::let_cxx_string!(filename = bin_filename);
    // let pc = pointcloud_reader::read_kitti_bin_file(filename);
    // let pc = binds::construct_pointcloud(pc);
    let calib = Calibration::new(calib_filename);
    let annos = read_annotation_file(label_filename).unwrap();
    let vertexes = get_vertexes_from_kitti_annos(annos, &calib);
    // let mut vertexes_vec = vec![];
    // for vertex in vertexes {
    //     for i in 0..vertex.nrows() {
    //         vertexes_vec.push(vertex[(i,0)]);
    //         vertexes_vec.push(vertex[(i,1)]);
    //         vertexes_vec.push(vertex[(i,2)]);
    //     }
    // }
    cxx::let_cxx_string!(title = "KITTI Visualizer");
    let mut visualizer = binds::get_visualizer(&title, autocxx::c_int(1600), autocxx::c_int(900));
    let pc = binds::read_pcd(&filename);
    visualizer = binds::add_geometry(visualizer, pc);
    for vertex in vertexes {
        let mut vertexes_vec = vec![];
        for i in 0..vertex.nrows() {
            vertexes_vec.push(vertex[(i,0)]);
            vertexes_vec.push(vertex[(i,1)]);
            vertexes_vec.push(vertex[(i,2)]);
        } 
        let lineset = binds::generate_boundingboxes_line_set(binds::Vertexes {data: vertexes_vec.try_into().expect("slice with incorrect length")});  
        visualizer = binds::add_lineset(visualizer, lineset);
    } 
    // let lineset = binds::generate_boundingboxes_line_set(binds::Vertexes {data: [5.0, 0.0, 0.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 5.0, 0.0, 5.0, 5.0, 5.0, 5.0, 0.0, 0.0, 5.0, 0.0, 5.0, 5.0]});
    // let visualizer = binds::add_lineset(visualizer, lineset);
    binds::run_visualizer(visualizer);
    // binds::visualize(pc);
}