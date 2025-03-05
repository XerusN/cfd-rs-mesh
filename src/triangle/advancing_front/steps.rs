use core::f64;
use std::f64::consts::PI;

use super::utils::*;
use cfd_rs_utils::{
    errors::MeshError,
    mesh::{indices::*, Modifiable2DMesh},
};
use nalgebra::{Point2, Vector2};

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

/// Based on J. Frysketig, 1994.
/// Returns a value in the normalized space.
pub fn ideal_node(
    mesh: &Modifiable2DMesh,
    base_edge_id: HalfEdgeIndex,
    element_size: f64,
) -> Option<Point2<f64>> {
    let base_edge = mesh.0.vertices_from_he(base_edge_id);
    let base_edge = [mesh.0.vertices(base_edge[0]), mesh.0.vertices(base_edge[1])];
    let space_normalization = NormalizedSpace::new(&mesh.0, base_edge_id);
    let base_edge = mesh
        .0
        .he_vector(base_edge_id)
        .to_normalized_space(&space_normalization);
    let prev_edge = (
        mesh.0.he_to_prev_he()[base_edge_id],
        mesh.0
            .he_vector(mesh.0.he_to_prev_he()[base_edge_id])
            .to_normalized_space(&space_normalization),
    );
    let next_edge = (
        mesh.0.he_to_next_he()[base_edge_id],
        mesh.0
            .he_vector(mesh.0.he_to_next_he()[base_edge_id])
            .to_normalized_space(&space_normalization),
    );
    let mut alpha = prev_edge.1 .0.angle(&base_edge.0).abs();
    let mut beta = next_edge.1 .0.angle(&base_edge.0).abs();
    if alpha > beta {
        let temp = alpha;
        alpha = beta;
        beta = temp;
    }

    if alpha < PI * 80. / 180. {
        return None;
    }

    if (alpha < PI * 91. / 180.) && (alpha > PI * 89. / 180.) {
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

pub fn node_validity_check(
    mesh: &Modifiable2DMesh,
    front: &[ParentIndex],
    element_size: f64,
    considered_point: ConsideredPoint,
) -> bool {
    let base_point = considered_point.coordinates(&mesh.0);

    let min = 0.67 * 0.67 * element_size * element_size;

    for parent in front {
        for node in mesh.0.vertices_from_parent(*parent) {
            let current_point = mesh.0.vertices(node);
            if nalgebra::distance_squared(&base_point, &current_point) < min {
                return false;
            }
        }
    }

    true
}

pub fn element_validity_check(
    mesh: &Modifiable2DMesh,
    front: &[ParentIndex],
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> bool {
    let a = mesh.0.vertices(mesh.0.vertices_from_he(base_edge)[0]);
    let b = mesh.0.vertices(mesh.0.vertices_from_he(base_edge)[1]);
    let c = considered_point.coordinates(&mesh.0);
    return triangle_intersect_front(&mesh.0, front, base_edge, considered_point);
}

pub fn node_suitability_check(
    mesh: &Modifiable2DMesh,
    front: &[ParentIndex],
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> bool {
    let tan_30 = (30. * f64::consts::PI / 180.).tan();

    let base_edge_vert = mesh.0.vertices_from_he(base_edge);
    let base_edge_vert = (
        mesh.0.vertices(base_edge_vert[0]),
        mesh.0.vertices(base_edge_vert[1]),
    );
    let base_edge_vec = mesh.0.he_vector(base_edge);
    let point = considered_point.coordinates(&mesh.0);

    let t = base_edge_vec.dot(&Vector2::new(
        point.x - base_edge_vert.0.x,
        point.y - base_edge_vert.0.y,
    )) / base_edge_vec.norm_squared();
    let theta = (t).max(t - 1.);
    let tan_a = Vector2::new(
        point.x - (base_edge_vert.0.x + t * base_edge_vec.x),
        point.y - (base_edge_vert.0.y + t * base_edge_vec.y),
    )
    .norm()
        / (theta * base_edge_vec.norm());
    if tan_a < tan_30 {
        return false;
    }

    let t = -base_edge_vec.dot(&Vector2::new(
        point.x - base_edge_vert.1.x,
        point.y - base_edge_vert.1.y,
    )) / base_edge_vec.norm_squared();
    let theta = (t).max(t - 1.);
    let tan_a = Vector2::new(
        point.x - (base_edge_vert.1.x - t * base_edge_vec.x),
        point.y - (base_edge_vert.1.y - t * base_edge_vec.y),
    )
    .norm()
        / (theta * base_edge_vec.norm());
    if tan_a < tan_30 {
        return false;
    }

    true
}

pub fn find_existing_candidates(
    mesh: &Modifiable2DMesh,
    front: &[ParentIndex],
    base_edge: HalfEdgeIndex,
    element_size: f64,
) -> Vec<ConsideredPoint> {
    let base_edge_vert = mesh.0.vertices_from_he(base_edge);
    let mid_base_edge = Point2::new(
        mesh.0.vertices(base_edge_vert[0]).x + mesh.0.vertices(base_edge_vert[1]).x,
        mesh.0.vertices(base_edge_vert[0]).y + mesh.0.vertices(base_edge_vert[1]).y,
    ) / 2.;

    let mut candidates = vec![];
    for &parent in front {
        for point in mesh.0.vertices_from_parent(parent) {
            if (base_edge_vert[0] != point) & (base_edge_vert[1] != point) {
                let vert = mesh.0.vertices(point);
                // Maybe *4 too small (*4*1.33*1.33 in the book but nor same lhs)
                if Vector2::new(vert.x - mid_base_edge.x, vert.y - mid_base_edge.y).norm_squared()
                    < element_size * element_size * 4.
                {
                    candidates.push(ConsideredPoint::OldPoint(point))
                }
            }
        }
    }

    candidates
}

pub fn create_trie_vertices(
    mesh: &Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    element_size: f64,
    ideal_node: Point2<f64>,
    number_trie_vertices: usize,
) -> Vec<ConsideredPoint> {
    let base_edge_vert = mesh.0.vertices_from_he(base_edge);
    let mid_base_edge = Point2::new(
        mesh.0.vertices(base_edge_vert[0]).x + mesh.0.vertices(base_edge_vert[1]).x,
        mesh.0.vertices(base_edge_vert[0]).y + mesh.0.vertices(base_edge_vert[1]).y,
    ) / 2.;

    let mut trie_vertices = vec![];
    for i in 0..(number_trie_vertices + 1) {
        trie_vertices.push(ConsideredPoint::NewPoint(Point2::new(
            mid_base_edge.x
                + (ideal_node.x - mid_base_edge.x) * ((i + 1) as f64)
                    / ((number_trie_vertices + 1) as f64),
            mid_base_edge.y
                + ((ideal_node.y - mid_base_edge.y) * (i + 1) as f64)
                    / ((number_trie_vertices + 1) as f64),
        )))
    }

    trie_vertices
}

pub fn add_element(
    mesh: &mut Modifiable2DMesh,
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) {
    todo!()
}
