mod binds;

use clap::Parser;

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
    cxx::let_cxx_string!(filename = args.dir);
    binds::visualize_pc(&filename);
}