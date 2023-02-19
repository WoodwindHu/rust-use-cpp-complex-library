# Rust use C++ Dynamic Library Demo

本项目给出了在 Mac 环境下 Rust 调用 C++ 库（Open3D）的示例。


This Demo show How Rust use C++ library in Mac OS.


## 打包
1. 下载 Open3D Binary 包
2. 将build.rs文件中的open3d_root修改为path/of/open3d
3. 将build.rs文件中的extra_clang_args修改为你电脑上的路径
4. `cargo run --release -- -d data/000000.bin -i 1` 

## Package
1. Download Open3D binary package
2. Change `open3d_root` in `build.rs` to your `path/of/open3d`
3. Change `extra_clang_args` in `build.rs` to your path of sdk
4. `cargo run --release -- -d data/000000.bin -i 1`