use core::f64;
use std::{f64::consts::PI, vec};

use super::utils::*;
use cfd_rs_utils::{
    errors::MeshError,
    mesh::{indices::*, Modifiable2DMesh},
};
use nalgebra::Point2;

#[cfg(test)]
mod test;

/// Refines the boundary up to the level specified by the element size.
pub fn refine_boundary(mesh: &mut Modifiable2DMesh, mesh_size: f64) {
    let mesh_size_squared = mesh_size * mesh_size;
    let mut i = 0;
    while i < mesh.0.he_len() {
        while mesh.0.he_vector(HalfEdgeIndex(i)).norm_squared() > mesh_size_squared {
            mesh.split_edge(HalfEdgeIndex(i), 0.5)
                .expect("Error when refining boundary");
        }
        i += 1;
    }
}

/// Selects the edge to be used.
/// For now most simple choice (only size based).
pub fn select_base_edge(mesh: &Modifiable2DMesh, front: &[ParentIndex]) -> HalfEdgeIndex {
    let edges_sizes_squared = mesh
        .0
        .he_from_parent(front[0])
        .iter()
        .map(|he| (*he, mesh.0.he_vector(*he).norm_squared()))
        .collect::<Vec<(HalfEdgeIndex, f64)>>();
    let mut current_min = 0;
    for (i, value) in edges_sizes_squared.iter().enumerate() {
        if value.1 < edges_sizes_squared[current_min].1 {
            current_min = i
        }
    }
    return edges_sizes_squared[current_min].0;
}

/// Will get proper definition later, for now the mesh size is considered to be constant so the base element size is constant too.
pub fn element_size(mesh: &Modifiable2DMesh, base_edge: HalfEdgeIndex, mesh_size: f64) -> f64 {
    mesh_size
}

pub fn new_element(
    mesh: &mut Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
) -> Result<(), MeshError> {
    todo!()
}

/// Based on J. Frysketig, 1994
/// Returns a value in the normalized space
pub fn ideal_node(
    mesh: &Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
) -> Option<Point2<f64>> {
    let base_edge = (base_edge, mesh.0.he_vector(base_edge).normalize());
    let prev_edge = (
        mesh.0.he_to_prev_he()[base_edge.0],
        mesh.0
            .he_vector(mesh.0.he_to_prev_he()[base_edge.0])
            .normalize(),
    );
    let next_edge = (
        mesh.0.he_to_next_he()[base_edge.0],
        mesh.0
            .he_vector(mesh.0.he_to_next_he()[base_edge.0])
            .normalize(),
    );
    let mut alpha = prev_edge.1.angle(&base_edge.1).abs();
    let mut beta = next_edge.1.angle(&base_edge.1).abs();
    if alpha > beta {
        let temp = alpha;
        alpha = beta;
        beta = temp;
    }

    if alpha < PI * 80. / 180. {
        return None;
    }

    if (alpha < PI * 91. / 180.) & (alpha > PI * 89. / 180.) {
        return Some(Point2::new(
            element_size * (PI / 4.).cos(),
            element_size * (PI / 4.).sin(),
        ));
    }

    let phi_a = alpha / (alpha / (PI / 3.)).round();
    let phi_b = beta / (beta / (PI / 3.)).round();

    return Some(
        Point2::new(phi_a.cos() - phi_b.cos(), phi_a.sin() - phi_b.sin()) * element_size / 2.,
    );
}

pub fn validity_check(
    mesh: &Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
    considered_point: ConsideredPoint,
) -> bool {
    todo!()
}

pub fn suitability_check(
    mesh: &Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
    considered_point: ConsideredPoint,
) -> bool {
    todo!()
}

pub fn find_existing_candidates(mesh: &Modifiable2DMesh, base_edge: HalfEdgeIndex, element_size: f64) -> Vec<ConsideredPoint> {
    todo!()
}

pub fn create_trie_vertices(
    mesh: &Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
) -> Vec<ConsideredPoint> {
    todo!()
}

pub fn add_element(
    mesh: &mut Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) {
    todo!()
}
