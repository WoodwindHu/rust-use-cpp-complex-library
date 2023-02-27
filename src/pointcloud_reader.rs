use std::{io::Read, fs::File};

use cxx::CxxVector;

/// read point cloud from a kitti-format binary file
fn read_kitti_bin_file(filename: &str) -> Vec<f32> {
    let mut file = File::open(filename).unwrap();
    // 计算文件大小
    let file_size = file.metadata().expect("unable to read metadata").len() as usize;

    // 读取整个文件到缓冲区
    let mut buffer = vec![0u8; file_size];
    file.read_exact(&mut buffer).expect("buffer overflow");

    // 将缓冲区中的数据解释为f32向量
    buffer.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])).collect::<Vec<_>>()
}
