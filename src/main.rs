use cfd_rs_utils::{
    boundary::*,
    mesh::{indices::*, Modifiable2DMesh, *},
};
use nalgebra::Point2;
use triangle::advancing_front::*;

pub mod triangle;

fn simple_mesh() -> Modifiable2DMesh {
    let parents = vec![Parent::Boundary(Boundary::NoSlip)];
    let vertices = vec![
        Point2::new(0.0, 0.0),
        Point2::new(1.0, 0.0),
        Point2::new(1.0, 1.0),
        Point2::new(0.0, 1.0),
    ];

    let edge_to_vertices_and_parent = vec![
        (VertexIndex(0), VertexIndex(1), ParentIndex(0)),
        (VertexIndex(1), VertexIndex(2), ParentIndex(0)),
        (VertexIndex(2), VertexIndex(3), ParentIndex(0)),
        (VertexIndex(3), VertexIndex(0), ParentIndex(0)),
    ];

    let mesh;

    unsafe {
        mesh = Modifiable2DMesh::new_from_boundary(vertices, edge_to_vertices_and_parent, parents);
    }

    mesh
}

fn main() {
    let mut mesh = simple_mesh();
    advancing_front(&mut mesh, 0.1, true).unwrap()
}
