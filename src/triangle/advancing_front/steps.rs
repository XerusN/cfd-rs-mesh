use core::f64;
use std::f64::consts::PI;

use super::utils::*;
use cfd_rs_utils::{
    errors::MeshError,
    mesh::{indices::*, Base2DMesh, Modifiable2DMesh},
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
    edges_sizes_squared[current_min].0
}

/// Will get proper definition later, for now the mesh size is considered to be constant so the base element size is constant too.
pub fn element_size(mesh_size: f64) -> f64 {
    mesh_size
}

pub fn new_element(
    mesh: &mut Modifiable2DMesh,
    front: &mut Vec<ParentIndex>,
    base_edge: HalfEdgeIndex,
    element_size: f64,
) -> Result<(), MeshError> {
    let ideal_nod;
    let mut valid_points = vec!();
    'ideal_node: {
        ideal_nod = ideal_node(mesh, base_edge, element_size);
        // println!("{:?}", ideal_nod);
        if ideal_nod.is_none() {
            break 'ideal_node;
        }

        let considered_point = ConsideredPoint::NewPoint(ideal_nod.expect("uh"));

        if !node_validity_check(mesh, front, element_size, considered_point, base_edge) {
            // println!("Node not valid");
            break 'ideal_node;
        }
        if !element_validity_check(mesh, front, base_edge, considered_point) {
            // println!("Element not valid");
            break 'ideal_node;
        }
        let tan_a = node_suitability_check(mesh, base_edge, considered_point);
        if tan_a < f64::tan(30.)  {
            valid_points.push((considered_point, tan_a));
            break 'ideal_node;
        }

        return add_element(mesh, front, base_edge, considered_point);
    }

    // Implement a choice based on the suitability criterion to enhance quality
    let existing_candidates = find_existing_candidates(mesh, front, base_edge, element_size);
    
    for point in existing_candidates {
        // if let ConsideredPoint::OldPoint(id) = point {
        //     println!("{:?} {:?}", point, mesh.0.vertices(id));
        // }
        'point: {
            if !node_validity_check(mesh, front, element_size, point, base_edge) {
                // println!("Node not valid");
                break 'point;
            }
            if !element_validity_check(mesh, front, base_edge, point) {
                // println!("Element not valid");
                break 'point;
            }
            let tan_a = node_suitability_check(mesh, base_edge, point);
            if tan_a < f64::tan(30.)  {
                valid_points.push((point, tan_a));
                break 'point;
            }

            return add_element(mesh, front, base_edge, point);
        }
    }

    // Implement a choice based on the suitability criterion to enhance quality
    if ideal_nod.is_none() {
        return Err(MeshError::NoElementCreatable(base_edge));
    }
    let trie_vertices = create_trie_vertices(mesh, base_edge, ideal_nod.expect("uh"), 4);
    for point in trie_vertices {
        'point: {
            // if let ConsideredPoint::NewPoint(coord) = point {
            //     println!("{:?} {:?}", point, coord);
            // }
            if !node_validity_check(mesh, front, element_size, point, base_edge) {
                // println!("Node not valid");
                break 'point;
            }
            if !element_validity_check(mesh, front, base_edge, point) {
                // println!("Element not valid");
                break 'point;
            }
            let tan_a = node_suitability_check(mesh, base_edge, point);
            if tan_a < f64::tan(30.)  {
                valid_points.push((point, tan_a));
                break 'point;
            }

            return add_element(mesh, front, base_edge, point);
        }
    }
    
    if valid_points.is_empty() {
        return Err(MeshError::NoElementCreatable(base_edge))
    } else {
        let mut max = (0, valid_points[0].1);
        for (i, valid) in valid_points.iter().enumerate() {
            if valid.1 > max.1 {
                max = (i, valid.1);
            }
        }
        return add_element(mesh, front, base_edge, valid_points[max.0].0)
    }

    
}

/// Based on J. Frysketig, 1994.
/// Returns a value in the normalized space.
pub fn ideal_node(
    mesh: &Modifiable2DMesh,
    base_edge_id: HalfEdgeIndex,
    element_size: f64,
) -> Option<Point2<f64>> {
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

    let mut alpha = prev_edge.1.0.angle(&base_edge.0).abs();
    let mut beta = (-next_edge.1.0).angle(&base_edge.0).abs();

    if alpha > beta {
        std::mem::swap(&mut alpha, &mut beta);
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

    Some(Point2::new(phi_a.cos() - phi_b.cos(), phi_a.sin() - phi_b.sin()) * element_size / 2.)
}

pub fn node_validity_check(
    mesh: &Modifiable2DMesh,
    front: &[ParentIndex],
    element_size: f64,
    considered_point: ConsideredPoint,
    base_edge: HalfEdgeIndex,
) -> bool {
    if let ConsideredPoint::OldPoint(old_point) = considered_point {
        
        if (mesh.0.vertices_from_he(mesh.0.he_to_next_he()[mesh.0.he_to_twin()[base_edge]])[1] == old_point) & (mesh.0.vertices_from_he(mesh.0.he_to_prev_he()[mesh.0.he_to_twin()[base_edge]])[0] == old_point) {
            return false
        }
        return true
    }
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
    triangle_intersect_front(&mesh.0, front, base_edge, considered_point)
}

pub fn node_suitability_check(
    mesh: &Modifiable2DMesh,
    // front: &[ParentIndex],
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> f64 {
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
        return tan_a;
    }
    let tan_a_old = tan_a;

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
        return tan_a;
    }
    
    tan_a.max(tan_a_old)
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
                // Maybe *4 too small (*4*1.33*1.33 in the book but not same lhs)
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
    // element_size: f64,
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

/// Once the quality and validity is checked modifies the data structure to add the new element
///
/// # Warning
///
/// Since the mesh structure is changed any value held before that might represent something else now
pub fn add_element(
    mesh: &mut Modifiable2DMesh,
    front: &mut Vec<ParentIndex>,
    base_edge: HalfEdgeIndex,
    considered_point: ConsideredPoint,
) -> Result<(), MeshError> {
    let point_id = match considered_point {
        ConsideredPoint::NewPoint(point) => {
            let new_parent;
            unsafe { new_parent = mesh.notching(base_edge, point)? };
            // If useless parent added it will get removed
            front.push(new_parent);
            clean_front(&mesh.0, front);
            return Ok(());
        }
        ConsideredPoint::OldPoint(point_id) => point_id,
    };

    // Double check this part
    if mesh.0.vertices_from_he(mesh.0.he_to_next_he()[base_edge])[1] == point_id {
        let front_parent = mesh.0.he_to_parent()[base_edge];
        let next_point = mesh.0.vertices_from_he(mesh.0.he_to_next_he()[base_edge])[1];
        let new_parent;
        unsafe {
            new_parent = mesh.trimming(
                (mesh.0.vertices_from_he(base_edge)[0], next_point),
                front_parent,
            )?;
        }
        front.push(new_parent);
        clean_front(&mesh.0, front);
        return Ok(());
    }
    if mesh.0.vertices_from_he(mesh.0.he_to_prev_he()[base_edge])[0] == point_id {
        let front_parent = mesh.0.he_to_parent()[base_edge];
        let next_point = mesh.0.vertices_from_he(mesh.0.he_to_prev_he()[base_edge])[0];
        let new_parent;
        unsafe {
            new_parent = mesh.trimming(
                (mesh.0.vertices_from_he(base_edge)[1], next_point),
                front_parent,
            )?;
        }
        front.push(new_parent);
        clean_front(&mesh.0, front);
        return Ok(());
    }

    // Only remains the case where both edge have to be created toward an old point, right?
    let front_parent = mesh.0.he_to_parent()[base_edge];
    let base_points = mesh.0.vertices_from_he(base_edge);

    // Relies on non-enforced behaviour of the trimming function! (old parent = parent from new he from vertices.0 to vertices.1)
    let new_parent1;
    let new_parent2;
    unsafe {
        new_parent1 = mesh.trimming((point_id, base_points[0]), front_parent)?;
    }
    unsafe {
        new_parent2 = mesh.trimming((point_id, base_points[1]), front_parent)?;
    }
    front.push(new_parent1);
    front.push(new_parent2);
    clean_front(&mesh.0, front);

    Ok(())
}

pub fn clean_front(mesh: &Base2DMesh, front: &mut Vec<ParentIndex>) {
    front.retain(|&parent| mesh.vertices_from_parent(parent).len() > 3);
}
