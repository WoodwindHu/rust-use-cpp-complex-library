use std::{fs::File, io::{BufReader, BufRead}, collections::HashMap, ops::DivAssign};

use nalgebra::{Matrix3x4, DMatrix, Matrix3};
/// Struct to reprensent every KITTI calibration.
#[derive(Debug, PartialEq)]
pub struct Calibration {
    P: DMatrix<f32>,
    V2C: DMatrix<f32>,
    C2V: DMatrix<f32>,
    R0: DMatrix<f32>,
}

impl Calibration{
    pub fn new(filename: String)->Self{
        let mut file = File::open(filename).expect("This file does not exist");
        let file = BufReader::new(file);
        let mut m:HashMap<String, Vec<String>> = HashMap::new();
        for line in file.lines() {
            let line = line.unwrap();
            let data: Vec<&str> = line.split_whitespace().collect();
            if data.len() == 0 {
                continue;
            }
            m.insert(data[0].to_string(), data[1..].into_iter().map(|x| x.to_string()).collect());
        }
        let P: Vec<f32> = m.get("P2:").unwrap().into_iter().map(|x| x.parse::<f32>().unwrap()).collect();
        let P = DMatrix::from_vec(4, 3, P).transpose();
        let mut V2C: Vec<f32> = m.get("Tr_velo_to_cam:").unwrap().into_iter().map(|x| x.parse::<f32>().unwrap()).collect();
        let V2C = DMatrix::from_vec(4, 3, V2C).transpose();
        let C2V = Calibration::inverse_rigid_trans(&V2C);
        let mut R0: Vec<f32> = m.get("R0_rect:").unwrap().into_iter().map(|x| x.parse::<f32>().unwrap()).collect();
        let R0 = DMatrix::from_vec(3, 3, R0).transpose();
        Self{P, V2C, C2V, R0}
    }
    fn inverse_rigid_trans(V2C:&DMatrix<f32>)->DMatrix<f32> {
        // Inverse a rigid body transform matrix (3x4 as [R|t])
        // [R'|-R't; 0|1]
        let mut inv_Tr = DMatrix::zeros(3, 4);
        inv_Tr.view_mut((0, 0), (3, 3)).copy_from(&V2C.view((0, 0), (3, 3)).transpose());
        inv_Tr.view_mut((0, 3), (3, 1)).copy_from(&(-V2C.view((0, 0), (3, 3)).transpose() * V2C.view((0, 3), (3, 1))));
        inv_Tr
    }
    fn cart2hom(pts_3d_velo:DMatrix<f32>)-> DMatrix<f32>{
        let mut pts_3d_velo = pts_3d_velo;
        let nrows = pts_3d_velo.nrows();
        let pts_3d_velo = pts_3d_velo.resize(nrows, 4, 1.);
        pts_3d_velo
    }

    // ===========================
    // ------- 3d to 3d ----------
    // ===========================

    pub fn project_velo_to_ref(&self, pts_3d_velo:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_velo = Calibration::cart2hom(pts_3d_velo);
        pts_3d_velo * self.V2C.transpose()
    }

    pub fn project_ref_to_velo(&self, pts_3d_ref:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_ref = Calibration::cart2hom(pts_3d_ref);
        pts_3d_ref * self.C2V.transpose()
    }

    pub fn project_rect_to_ref(&self, pts_3d_rect:DMatrix<f32>)-> DMatrix<f32>{
        (self.R0.clone().try_inverse().unwrap() * pts_3d_rect.transpose()).transpose()
    }

    pub fn project_ref_to_rect(&self, pts_3d_ref:DMatrix<f32>)-> DMatrix<f32>{
        (&self.R0 * pts_3d_ref.transpose()).transpose()
    }

    pub fn project_rect_to_velo(&self, pts_3d_rect:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_ref = self.project_rect_to_ref(pts_3d_rect);
        self.project_ref_to_velo(pts_3d_ref)
    }

    pub fn project_velo_to_rect(&self, pts_3d_velo:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_ref = self.project_velo_to_ref(pts_3d_velo);
        self.project_ref_to_rect(pts_3d_ref)
    }

    // ===========================
    // ------- 3d to 2d ----------
    // ===========================
    pub fn project_rect_to_image(&self, pts_3d_rect:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_rect = Calibration::cart2hom(pts_3d_rect);
        let mut pts_2d = pts_3d_rect * self.P.transpose();
        let tmp = pts_2d.column(2).into_owned();
        pts_2d.column_mut(0).component_div_assign(&tmp);
        pts_2d.column_mut(1).component_div_assign(&tmp);
        pts_2d
    }

    pub fn project_velo_to_image(&self, pts_3d_velo:DMatrix<f32>)-> DMatrix<f32>{
        let pts_3d_rect = self.project_velo_to_rect(pts_3d_velo);
        self.project_rect_to_image(pts_3d_rect)
    }

    fn project_8p_to_4p(&self, pts_2d:DMatrix<f32>)-> Vec<f32>{
        let mut pts_2d = pts_2d;
        let x0 = pts_2d.column(0).min();
        let x1 = pts_2d.column(0).max();
        let y0 = pts_2d.column(1).min();
        let y1 = pts_2d.column(1).max();
        let x0 = x0.max(0.);
        let y0 = y0.max(0.);
        vec![x0, y0, x1, y1]
    }
    
    fn project_velo_to_4p(&self, pts_3d_velo:DMatrix<f32>)-> Vec<f32> {
        let pts_2d_velo = self.project_velo_to_image(pts_3d_velo);
        self.project_8p_to_4p(pts_2d_velo)
    }

    // ===========================
    // ------- 2d to 3d ----------
    // ===========================
    fn project_image_to_rect(&self, uv_depth:DMatrix<f32>)-> DMatrix<f32>{
        let c_u = self.P[(0, 2)];
        let c_v = self.P[(1, 2)];
        let f_u = self.P[(0, 0)];
        let f_v = self.P[(1, 1)];
        let b_x = self.P[(0, 3)] / (-f_u);
        let b_y = self.P[(1, 3)] / (-f_v);
        let x = (uv_depth.column(0) - DMatrix::from_vec(uv_depth.ncols(),  1, vec![c_u; uv_depth.ncols()])) * uv_depth.column(2) / f_u + DMatrix::from_vec(uv_depth.ncols(),  1, vec![b_x; uv_depth.ncols()]);
        let y = (uv_depth.column(1) - DMatrix::from_vec(uv_depth.ncols(),  1, vec![c_v; uv_depth.ncols()])) * uv_depth.column(2) / f_v + DMatrix::from_vec(uv_depth.ncols(),  1, vec![b_y; uv_depth.ncols()]);
        let mut pts_3d_rect = DMatrix::zeros(3, uv_depth.ncols());
        pts_3d_rect.column_mut(0).copy_from(&x);
        pts_3d_rect.column_mut(1).copy_from(&y);
        pts_3d_rect.column_mut(2).copy_from(&uv_depth.column(2));
        pts_3d_rect
    }

    fn project_image_to_velo(&self, uv_depth:DMatrix<f32>)-> DMatrix<f32> {
        let pts_3d_rect = self.project_image_to_rect(uv_depth);
        self.project_rect_to_velo(pts_3d_rect)
    }

    // def project_depth_to_velo(self, depth, constraint_box=True):
    //     depth_pt3d = get_depth_pt3d(depth)
    //     depth_UVDepth = np.zeros_like(depth_pt3d)
    //     depth_UVDepth[:, 0] = depth_pt3d[:, 1]
    //     depth_UVDepth[:, 1] = depth_pt3d[:, 0]
    //     depth_UVDepth[:, 2] = depth_pt3d[:, 2]
    //     depth_pc_velo = self.project_image_to_velo(depth_UVDepth)
    //     if constraint_box:
    //         depth_box_fov_inds = (
    //             (depth_pc_velo[:, 0] < cbox[0][1])
    //             & (depth_pc_velo[:, 0] >= cbox[0][0])
    //             & (depth_pc_velo[:, 1] < cbox[1][1])
    //             & (depth_pc_velo[:, 1] >= cbox[1][0])
    //             & (depth_pc_velo[:, 2] < cbox[2][1])
    //             & (depth_pc_velo[:, 2] >= cbox[2][0])
    //         )
    //         depth_pc_velo = depth_pc_velo[depth_box_fov_inds]
    //     return depth_pc_velo
}