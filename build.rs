fn main() -> miette::Result<()> {
    let open3d_root = "/Users/potapo/open3d";
    let path_src = std::path::PathBuf::from("src");
    let path_open3d_include = std::path::PathBuf::from(format!("{}/{}", open3d_root, "include"));
    let path_open3d_lib = std::path::PathBuf::from(format!("{}/{}", open3d_root, "lib"));
    let path_open3d_3rdparty = std::path::PathBuf::from(format!("{}/{}", open3d_root, "include/open3d/3rdparty"));


    println!("cargo:rustc-link-search={}", path_open3d_lib.to_string_lossy());
    println!("cargo:rustc-link-arg=-Wl,-rpath,{}", path_open3d_lib.to_string_lossy());
    println!("cargo:rustc-link-lib=dylib=Open3D");
    let mut b = autocxx_build::Builder::new("src/binds.rs", [&path_src, &path_open3d_include, &path_open3d_3rdparty])
                        .extra_clang_args(&["-isysroot", "/Library/Developer/CommandLineTools/SDKs/MacOSX13.1.sdk"])
                        .build()?;

    b.flag_if_supported("-std=c++14").compile("bridge-open3d");

    println!("cargo:rerun-if-changed=src/main.rs");
    println!("cargo:rerun-if-changed=src/open3d_wrapper.h");
    println!("cargo:rerun-if-changed=src/binds.rs");

    Ok(())
}
