//! KITTI Annotation file parsing.
use anyhow::Result;
use nalgebra::DMatrix;
use nalgebra::Point3;
use std::ffi::OsStr;
use std::fs::File;
use std::io::prelude::*;
use std::io::BufReader;
use std::path::Path;

use crate::calibration::Calibration;
/// Struct to reprensent every KITTI annotation.
#[derive(Debug, Default, PartialEq)]
pub struct KittiAnnotation {
    /// Describes the type of object:
    /// 'Car', 'Van', 'Truck',
    /// 'Pedestrian', 'Person_sitting',
    /// 'Cyclist', 'Tram',
    /// 'Misc' or 'DontCare'.
    pub category: String,
    /// Float from 0 (non-truncated)
    /// to 1 (truncated), where truncated refers
    /// to the object leaving image boundaries
    pub truncation: f32,
    /// Integer (0,1,2,3) indicating occlusion state:
    /// 0 = fully visible,
    /// 1 = partly occluded,
    /// 2 = largely occluded,
    /// 3 = unknown
    pub occlusion: i16,
    /// Observation angle of object, ranging [-pi..pi]
    pub alpha: f32,
    /// 2D bounding box xmin of object in the image
    pub xmin: f32,
    /// 2D bounding box ymin of object in the image
    pub ymin: f32,
    /// 2D bounding box xmax of object in the image
    pub xmax: f32,
    /// 2D bounding box ymax of object in the image
    pub ymax: f32,
    /// 3D object dimensions  height(in meters)
    pub h: f32,
    /// 3D object dimensions width (in meters)
    pub w: f32,
    /// 3D object dimensions length (in meters)
    pub l: f32,
    /// 3D object location x in camera coordinates (in meters)
    pub x: f32,
    /// 3D object location y in camera coordinates (in meters)
    pub y: f32,
    /// 3D object location z in camera coordinates (in meters)
    pub z: f32,
    /// Rotation ry around Y-axis in camera coordinates [-pi..pi]
    pub ry: f32,
    /// Score indicating confidence in detection [0..1]
    /// If coming from gound truth score is 1.0
    pub score: f32,
}

impl KittiAnnotation {
    /// Create a KittiAnnotation
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        category: String,
        truncation: f32,
        occlusion: i16,
        alpha: f32,
        xmin: f32,
        ymin: f32,
        xmax: f32,
        ymax: f32,
        height: f32,
        width: f32,
        length: f32,
        xc: f32,
        yc: f32,
        zc: f32,
        ry: f32,
        score: f32,
    ) -> Self {
        Self {
            category,
            truncation,
            occlusion,
            alpha,
            xmin,
            ymin,
            xmax,
            ymax,
            h: height,
            w: width,
            l: length,
            x: xc,
            y: yc,
            z: zc,
            ry,
            score,
        }
    }
    /// Return the 2D BoundingBox in image coordinates system
    pub fn get_2d_bounding_box(self) -> [f32; 4] {
        [self.xmin, self.ymin, self.xmax, self.ymax]
    }
    /// Return the 3D BoundingBox in the Lidar coordinates system
    pub fn get_3d_bounding_box(self) -> [f32; 7] {
        [self.x, self.y, self.z, self.h, self.w, self.l, self.ry]
    }
}

/// Parse a KITTI annotation file describe in the DevKit
pub fn read_annotation_file(kitti_annotations_path: String) -> Result<Vec<KittiAnnotation>> {
    let extension = Path::new(&kitti_annotations_path).extension();
    if extension != Some(OsStr::new("txt")) {
        panic!(
            "KITTI annotation file are in txt format and it received an got {:?}.",
            extension
        );
    }
    let file = File::open(kitti_annotations_path).expect("This file does not exist");
    let file = BufReader::new(file);
    let mut annotation: Vec<KittiAnnotation> = vec![];
    for line in file.lines() {
        let line = line?;
        let data: Vec<&str> = line.split_whitespace().collect();
        if data[0] != "DontCare" {
            let anno = KittiAnnotation::new(
                data[0].to_string(),
                data[1].parse()?,
                data[2].parse()?,
                data[3].parse()?,
                data[4].parse()?,
                data[5].parse()?,
                data[6].parse()?,
                data[7].parse()?,
                data[8].parse()?,
                data[9].parse()?,
                data[10].parse()?,
                data[11].parse()?,
                data[12].parse()?,
                data[13].parse()?,
                data[14].parse()?,
                1.0,
            );
            annotation.push(anno);
        }
    }
    Ok(annotation)
}

fn sort_vertexes(vertexes:  &mut DMatrix<f32>) {
    // Draw 3d bounding box
    //   6 -------- 7
    //  /|         /|
    // 4 -------- 5 .
    // | |        | |
    // . 2 -------- 3
    // |/         |/
    // 0 -------- 1
    let mut bbox: Vec<Point3<f32>> = vec![];
    for i in 0..8 {
        bbox.push(Point3::new(vertexes[(i, 0)], vertexes[(i, 1)], vertexes[(i, 2)]));
    }
    bbox.sort_by(|a, b| a[2].partial_cmp(&b[2]).unwrap());
    bbox[0..4].sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());    
    bbox[0..2].sort_by(|a, b| a[1].partial_cmp(&b[1]).unwrap());
    bbox[2..4].sort_by(|a, b| a[1].partial_cmp(&b[1]).unwrap());
    bbox[4..8].sort_by(|a, b| a[0].partial_cmp(&b[0]).unwrap());
    bbox[4..6].sort_by(|a, b| a[1].partial_cmp(&b[1]).unwrap());
    bbox[6..8].sort_by(|a, b| a[1].partial_cmp(&b[1]).unwrap());
    for i in 0..8 {
        vertexes[(i, 0)] = bbox[i][0];
        vertexes[(i, 1)] = bbox[i][1];
        vertexes[(i, 2)] = bbox[i][2];
    }
}

fn get_vertexes_from_kitti_anno(anno: KittiAnnotation, calib: &Calibration)->DMatrix<f32> {
    let loc = calib.project_rect_to_velo(DMatrix::from_vec(1, 3, vec![anno.x, anno.y, anno.z]));
    let mut vertex = DMatrix::from_vec(8, 3, vec![0.0f32;24]);
    let rot = DMatrix::from_vec(2, 2, vec![anno.ry.cos(), -anno.ry.sin(), anno.ry.sin(), anno.ry.cos()]).transpose();
    let shift_xy = DMatrix::from_vec(2, 4, vec![anno.w/2.0,  anno.l/2.0, anno.w/2.0, -anno.l/2.0, -anno.w/2.0,  anno.l/2.0, -anno.w/2.0, -anno.l/2.0]).transpose();
    let shift_xy = shift_xy * rot;
    vertex.view_mut((0,0), (4,1)).copy_from(&(shift_xy.view((0,0), (4,1)) + DMatrix::from_vec(4, 1, vec![loc[(0,0)];4])));
    vertex.view_mut((0,1), (4,1)).copy_from(&(shift_xy.view((0,1), (4,1)) + DMatrix::from_vec(4, 1, vec![loc[(0,1)];4])));
    vertex.view_mut((0,2), (4,1)).copy_from(&(DMatrix::from_vec(4, 1, vec![loc[(0,2)];4])));
    let tmp = vertex.view((0,0), (4,2)).into_owned();
    vertex.view_mut((4,0), (4,2)).copy_from(&tmp);
    vertex.view_mut((4,2), (4,1)).copy_from(&(DMatrix::from_vec(4, 1, vec![loc[(0,2)]+anno.h;4])));
    sort_vertexes(&mut vertex);
    vertex
}

pub fn get_vertexes_from_kitti_annos(annos: Vec<KittiAnnotation>, calib: &Calibration)->Vec<DMatrix<f32>> {
    let mut vertexes = vec![];
    for anno in annos {
        let vertex = get_vertexes_from_kitti_anno(anno, calib);
        vertexes.push(vertex);
    }
    vertexes
}