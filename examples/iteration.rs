use cavalier_contours::polyline::{PlineSource, PlineSourceMut, Polyline};

fn main() {
    vertex_iteration();
    segment_iteration();
    index_wrapping();
}

fn vertex_iteration() {
    println!("Testing vertex iteration methods...");

    // Create a polyline with mixed vertex types
    let mut pline = Polyline::<f64>::new();
    pline.add(0.0, 0.0, 0.0); // Regular vertex
    pline.add(10.0, 0.0, 0.5); // Arc vertex (bulge = 0.5)
    pline.add(20.0, 10.0, 0.0); // Regular vertex
    pline.set_is_closed(true);

    assert_eq!(pline.vertex_count(), 3, "Polyline should have 3 vertices");

    // Method 1: Iterator access
    let vertices: Vec<_> = pline.iter_vertexes().collect();
    assert_eq!(
        vertices[0].x, 0.0,
        "First vertex x-coordinate should be 0.0"
    );
    assert_eq!(
        vertices[1].bulge, 0.5,
        "Second vertex should have bulge 0.5"
    );

    // Method 2: Indexed access with get()
    let vertex = pline.get(1).unwrap();
    assert_eq!(vertex.x, 10.0, "Second vertex x-coordinate should be 10.0");
    assert_eq!(vertex.bulge, 0.5, "Second vertex should have bulge 0.5");

    // Method 3: Direct access with at() (panics if out of bounds)
    let vertex = pline.at(2);
    assert_eq!(vertex.x, 20.0, "Third vertex x-coordinate should be 20.0");

    // Filter arc vertices
    let arc_count = pline.iter_vertexes().filter(|v| v.bulge != 0.0).count();
    assert_eq!(arc_count, 1, "Should have exactly 1 arc vertex");

    println!("Vertex iteration methods completed successfully!");
}

fn segment_iteration() {
    println!("Testing segment iteration methods...");

    let mut pline = Polyline::<f64>::new();
    pline.add(0.0, 0.0, 0.0); // Line segment to next
    pline.add(10.0, 0.0, 0.5); // Arc segment to next
    pline.add(20.0, 10.0, 0.0); // Line segment to next
    pline.set_is_closed(true);

    // Basic segment iteration
    let segments: Vec<_> = pline.iter_segments().collect();
    assert_eq!(segments.len(), 3, "Closed polyline should have 3 segments");
    assert_eq!(segments[0].0.bulge, 0.0, "First segment should be a line");
    assert_eq!(segments[1].0.bulge, 0.5, "Second segment should be an arc");
    assert_eq!(segments[2].0.bulge, 0.0, "Third segment should be a line");

    // Segment index iteration
    let indexes: Vec<_> = pline.iter_segment_indexes().collect();
    assert_eq!(
        indexes[0],
        (0, 1),
        "First segment should connect vertex 0 to 1"
    );
    assert_eq!(
        indexes[1],
        (1, 2),
        "Second segment should connect vertex 1 to 2"
    );
    assert_eq!(
        indexes[2],
        (2, 0),
        "Third segment should wrap to start (closed)"
    );

    // Closed vs open segment counts
    let mut open_pline = pline.clone();
    open_pline.set_is_closed(false);
    assert_eq!(
        pline.segment_count(),
        3,
        "Closed polyline should have n segments"
    );
    assert_eq!(
        open_pline.segment_count(),
        2,
        "Open polyline should have n-1 segments"
    );

    println!("Segment iteration methods completed successfully!");
}

fn index_wrapping() {
    println!("Testing index wrapping utilities...");

    let mut pline = Polyline::<f64>::new();
    pline.add(0.0, 0.0, 0.0); // Index 0
    pline.add(10.0, 0.0, 0.0); // Index 1
    pline.add(10.0, 10.0, 0.0); // Index 2
    pline.add(0.0, 10.0, 0.0); // Index 3
    pline.set_is_closed(true);

    // Test wrapping behavior
    assert_eq!(
        pline.next_wrapping_index(3),
        0,
        "Next index after last should wrap to start"
    );
    assert_eq!(
        pline.prev_wrapping_index(0),
        3,
        "Previous index before first should wrap to end"
    );
    assert_eq!(
        pline.fwd_wrapping_dist(1, 3),
        2,
        "Forward distance from index 1 to 3 should be 2"
    );
    assert_eq!(
        pline.fwd_wrapping_index(2, 3),
        1,
        "Forward wrapping index with offset should be 1"
    );

    // Practical neighbor access
    let prev_idx = pline.prev_wrapping_index(0);
    let next_idx = pline.next_wrapping_index(0);
    let neighbors = (pline.at(prev_idx), pline.at(0), pline.at(next_idx));
    assert_eq!(
        neighbors.0.x, 0.0,
        "Previous neighbor should have x-coordinate 0.0"
    );
    assert_eq!(
        neighbors.2.x, 10.0,
        "Next neighbor should have x-coordinate 10.0"
    );

    println!("Index wrapping utilities completed successfully!");
}
