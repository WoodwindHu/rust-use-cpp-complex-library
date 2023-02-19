use autocxx::prelude::*;

include_cpp!(
    #include "open3d_wrapper.h"
    safety!(unsafe_ffi)
    generate!("visualize_pc")
);


pub use ffi::visualize_pc;