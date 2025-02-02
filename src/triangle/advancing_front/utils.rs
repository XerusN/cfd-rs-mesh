use cfd_rs_utils::mesh::indices::*;
use nalgebra::{Point2, Vector2};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConsideredPoint {
    NewPoint(Point2<f64>),
    OldPoint(VertexIndex),
}

#[derive(Clone, Debug, PartialEq)]
pub struct NormalizedSpace {
    pub mesh_size: f64,
    pub stretch_factor: f64,
    pub stretch_direction: Point2<f64>,
}

pub trait SpaceNormalize {
    fn normalize(&mut self, normalized_space: NormalizedSpace);

    fn denormalize(&mut self, normalized_space: NormalizedSpace);
}

impl SpaceNormalize for Point2<f64> {
    fn normalize(&mut self, normalized_space: NormalizedSpace) {
        todo!()
    }

    fn denormalize(&mut self, normalized_space: NormalizedSpace) {
        todo!()
    }
}

impl SpaceNormalize for Vector2<f64> {
    fn normalize(&mut self, normalized_space: NormalizedSpace) {
        todo!()
    }

    fn denormalize(&mut self, normalized_space: NormalizedSpace) {
        todo!()
    }
}

pub fn triangle_intersect_front(
    front: ParentIndex,
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> bool {
    todo!()
}
