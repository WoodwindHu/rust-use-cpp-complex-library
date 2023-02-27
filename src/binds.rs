use autocxx::prelude::*;


include_cpp!(
    #include "open3d_wrapper.h"
    safety!(unsafe_ffi)
    generate!("read_pcd")
    generate!("visualize")
    generate!("get_visualizer")
    generate!("add_geometry")
    generate!("run_visualizer")
    generate_pod!("Vertexes")
    generate!("generate_boundingboxes_line_set")
    generate!("add_lineset")
);


#[autocxx::extern_rust::extern_rust_function]
fn test_slice(slice: Vec<f32>) {
    println!("test_slice: {:?}", slice);
}


pub use ffi::read_pcd;
pub use ffi::visualize;
pub use ffi::get_visualizer;
pub use ffi::add_geometry;
pub use ffi::run_visualizer;
pub use ffi::Vertexes;
pub use ffi::generate_boundingboxes_line_set;
pub use ffi::add_lineset;