use super::*;
use cfd_rs_utils::{boundary::Boundary, mesh::{Modifiable2DMesh, Parent}};

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
// No tesst for points yet
fn space_normalization_test() {
    
    let mesh = simple_mesh();
    let he = HalfEdgeIndex(2);
    
    let space = NormalizedSpace::new(&mesh.0, he);
    let he_vec = mesh.0.he_vector(he);
    let he_vec_norm = he_vec.to_normalized_space(&space);
    let prev = mesh.0.he_to_prev_he()[he];
    let prev_vec = mesh.0.he_vector(prev);
    let prev_vec_norm = prev_vec.to_normalized_space(&space);
    println!("prev {:?} {:?}", prev, prev_vec);
    
    println!("prev {:?} {:?}", he_vec_norm, prev_vec_norm);
    // Checks that the angle is conserved (invalid if stretching is implemented)
    if he_vec.angle(&prev_vec) != he_vec_norm.0.angle(&prev_vec_norm.0) {
        panic!("Angle not conserved by normalization: {:?} and normalized {:?}", he_vec.angle(&prev_vec), he_vec_norm.0.angle(&prev_vec_norm.0));
    };
    
    let vec = Vector2::new(0.5, 1.8);
    let vec_norm = vec.to_normalized_space(&space);
    // Checks that the angle is conserved (invalid if stretching is implemented)
    println!("vec {:?} {:?}", he_vec_norm, vec_norm);
    if he_vec.angle(&vec) != he_vec_norm.0.angle(&vec_norm.0) {
        panic!("Angle not conserved by normalization: {:?} and normalized {:?}", he_vec.angle(&vec), he_vec_norm.0.angle(&vec_norm.0));
    };
    
    // Checks that the vector is the same after normalization and inverse
    if vec != Vector2::from_normalized_space(&vec_norm, &space) {
        panic!("Not equal after normalization and inverse: {:?} and {:?}", vec, Vector2::from_normalized_space(&vec_norm, &space));
    };
    
}