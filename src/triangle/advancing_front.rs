use cfd_rs_utils::*;
use errors::MeshError;
use mesh::indices::*;
use mesh::Modifiable2DMesh;
use steps::*;
use utils::*;

pub mod steps;
pub mod utils;

/// Advancing front algorithm.
/// The boundaries need to be a closed loop.
/// For now only single boundaries can be used safely (no hole and only one Cell).
///
/// # Safety
///
/// The function is at a really early stage of its implementation, so be careful with the input.
pub fn advancing_front(mesh: &mut Modifiable2DMesh) -> Result<(), MeshError> {
    let mut first_cell = None;

    for (i, parent) in mesh.0.parents().iter().enumerate() {
        match parent {
            mesh::Parent::Cell => first_cell = Some(ParentIndex(i)),
            _ => (),
        }
    }

    // With how the function are defined, this should remain the cell containing each edge from the front
    let first_cell = match first_cell {
        Some(value) => value,
        None => return Err(MeshError::WrongMeshInitialisation),
    };

    loop {
        println!("Front size : {:?}", mesh.0.he_from_parent(first_cell).len());

        if mesh.0.he_from_parent(first_cell).len() <= 3 {
            break;
        }
    }

    Ok(())
}
