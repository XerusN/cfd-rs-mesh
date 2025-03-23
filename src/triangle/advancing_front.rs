use cfd_rs_utils::*;
use errors::MeshError;
use mesh::indices::*;
use mesh::Modifiable2DMesh;
use steps::*;

pub mod steps;
pub mod utils;

/// Advancing front algorithm.
/// The boundaries need to be a closed loop.
/// For now only single boundaries can be used safely (no hole and only one Cell).
///
/// # Safety
///
/// The function is at a really early stage of its implementation, so be careful with the input.
///
/// Only one cell is expected for the input.
pub fn advancing_front(
    mesh: &mut Modifiable2DMesh,
    element_size: f64,
    step_output: bool,
) -> Result<(), MeshError> {
    let mut first_cell = None;

    for (i, parent) in mesh.0.parents().iter().enumerate() {
        if parent == &mesh::Parent::Cell {
            first_cell = Some(ParentIndex(i))
        }
    }

    // With how the function are defined, this should remain the cell containing each edge from the front
    let first_cell = match first_cell {
        Some(value) => value,
        None => return Err(MeshError::WrongMeshInitialisation),
    };

    let mut front = vec![first_cell];

    refine_boundary(mesh, element_size);

    let mut i = 0;
    if step_output {
        mesh.0
            .export_vtk(format!("./output/advancing_{}.vtk", i).as_str())
            .expect("");
    }
    loop {
        i += 1;
        println!(
            "{:?}) Front size : {:?}",
            i,
            mesh.0.he_from_parent(first_cell).len()
        );

        if front.is_empty() {
            break;
        }

        let mut base_edge = select_base_edge(mesh, &front);
        let first_edge = base_edge;
        loop {
            let result = new_element(mesh, &mut front, base_edge, element_size);
            if let Err(MeshError::NoElementCreatable(_)) = result {
                base_edge = mesh.0.he_to_next_he()[base_edge];
                if base_edge == first_edge {
                    return Err(MeshError::NoElementCreatable(base_edge));
                }
            } else {
                break;
            }
        }
        //println!("mesh: {:?}", mesh);
        //println!("base edge: {:?} {:?} {:?} {:?}", base_edge, mesh.0.vertices(mesh.0.vertices_from_he(base_edge)[0]), mesh.0.vertices(mesh.0.vertices_from_he(base_edge)[1]), mesh.0.he_to_parent()[base_edge]);
        
        println!("Check mesh: {:?}", mesh.0.check_mesh());
        
        if step_output {
            mesh.0
                .export_vtk(format!("./output/advancing_{}.vtk", i).as_str())
                .expect("");
        }
        
        
    }

    Ok(())
}
