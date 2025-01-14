use cfd_rs_utils::mesh::indices::*;
use nalgebra::Point2;

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConsideredPoint {
    NewPoint(Point2<f64>),
    OldPoint(VertexIndex),
}

#[derive(Clone, Debug, PartialEq)]
pub struct NormalizedSpace {
    mesh_size: f64,
    stretch_factor: f64,
    stretch_direction: Point2<f64>,
}

pub fn normalize(normalized_space: NormalizedSpace, vector: Point2<f64>) -> Point2<f64> {
    todo!()
}

pub fn triangle_intersect_front(
    front: ParentIndex,
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> bool {
    todo!()
}
