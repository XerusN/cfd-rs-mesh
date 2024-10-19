use cfd_rs_utils::*;

/// Advancing front algorithm.
/// The boundaries need to be locked, unexpected things might happen if some edges are shared between boundaries.
/// Same if some boundaries are defined the wrong way (clockwise or not defines the direction of expansion)
pub fn advancing_front(boundaries: &Vec<Boundary2D>, edges: Vec<Edge2D>, nodes: Vec<Point2<f64>>) -> MeshBlock2D<Triangle> {
    for boundary in boundaries {
        assert!(boundary.status(), "A Boundary is not closed");
    }
    
    let mesh = MeshBlock2D::<Triangle>::new(nodes, edges, Vec::new());
    
    //let mut edge_stack = 
    
    mesh
}