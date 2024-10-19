use cfd_rs_utils::*;

pub fn advancing_front(boundaries: &Vec<Boundary2D>) -> MeshBlock2D<Triangle> {
    for boundary in boundaries {
        assert!(boundary.status(), "A Boundary is not closed");
    }
    
    
    
    
    
    MeshBlock2D::new(nodes, edges, cells)
}