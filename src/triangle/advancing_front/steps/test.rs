use super::*;
use cfd_rs_utils::{boundary::Boundary, mesh::Parent};

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

#[test]
fn refine_boundary_test1() {
    let mut mesh = simple_mesh();
    let base_len = mesh.0.he_len();
    refine_boundary(&mut mesh, 0.09);
    assert!(mesh.0.he_len() > base_len * 5);
}