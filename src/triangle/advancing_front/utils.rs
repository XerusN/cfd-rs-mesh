use cfd_rs_utils::mesh::indices::*;
use nalgebra::{Point2, Vector2};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConsideredPoint {
    NewPoint(Point2<f64>),
    OldPoint(VertexIndex),
}

/// Space Normalization for isotropic mesh control
#[derive(Clone, Debug, PartialEq)]
pub struct NormalizedSpace {
    pub mesh_size: f64,
    pub base_edge: [Point2<f64>; 2],
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct SpaceNormalized<T>(pub T);

pub trait SpaceNormalize {
    
    fn to_normalized_space<S>(&self, normalized_space: &NormalizedSpace) -> SpaceNormalized<S>;

    fn from_normalized_space<S>(
        coordinates: SpaceNormalized<&S>,
        normalized_space: &NormalizedSpace,
    ) -> Self;
}

impl SpaceNormalize for Point2<f64> {
    fn to_normalized_space<S>(&self, normalized_space: &NormalizedSpace) -> SpaceNormalized<S> {
        todo!()
    }

    fn from_normalized_space<S>(
        coordinates: SpaceNormalized<&S>,
        normalized_space: &NormalizedSpace,
    ) -> Self {
        todo!()
    }
}

impl SpaceNormalize for Vector2<f64> {
    fn to_normalized_space<S>(&self, normalized_space: &NormalizedSpace) -> SpaceNormalized<S> {
        todo!()
    }

    fn from_normalized_space<S>(
        coordinates: SpaceNormalized<&S>,
        normalized_space: &NormalizedSpace,
    ) -> Self {
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
