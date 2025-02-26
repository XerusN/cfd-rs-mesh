use cfd_rs_utils::mesh::{indices::*, Base2DMesh};
use nalgebra::{Point2, Vector2};

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ConsideredPoint {
    NewPoint(Point2<f64>),
    OldPoint(VertexIndex),
}

impl ConsideredPoint {
    pub fn coordinates(&self, mesh: &Base2DMesh) -> Point2<f64> {
        match self {
            ConsideredPoint::NewPoint(point) => *point,
            ConsideredPoint::OldPoint(point_id) => mesh.vertices(*point_id),
        }
    }
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

pub fn edge_intersect_parent(
    mesh: &Base2DMesh,
    front: &[ParentIndex],
    edge: &[Point2<f64>],
) -> bool {
    todo!()
}

pub fn triangle_intersect_front(
    mesh: &Base2DMesh,
    front: &[ParentIndex],
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> bool {
    todo!()
}

pub fn edge_intersect_triangle(triangle: &[Point2<f64>], edge: &[Point2<f64>]) -> bool {
    // 1. If both point are on the same negative half-edge => no intersection
    for i in 0..3 {
        if half_plane(&[triangle[i], triangle[(i + 1) % 3]], &edge[0]) < 0.
            && half_plane(&[triangle[i], triangle[(i + 1) % 3]], &edge[1]) < 0.
        {
            return false;
        }
    }

    false
}

/// Determines if a point is on the positive or negative half-plane of an edge
pub fn half_plane(edge: &[Point2<f64>], point: &Point2<f64>) -> f64 {
    return (edge[1].x - edge[0].x) * (point.y - edge[0].y)
        - (edge[1].y - edge[0].y) * (point.x - edge[0].x);
}

/// Tells if a point is in a triangle, the triangle points must be in the trigonometric order.
pub fn point_in_triangle(triangle: &[Point2<f64>], point: &Point2<f64>) -> bool {
    
    // A point is in a triangle if and only if he is on the 3 positive half-planes
    for i in 0..3 {
        if half_plane(&[triangle[i], triangle[(i + 1) % 3]], point) <= 0.
        {
            return false
        }
    }
    
    return true
}
