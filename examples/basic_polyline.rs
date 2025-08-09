use cavalier_contours::polyline::{
    PlineCreation, PlineSource, PlineSourceMut, PlineVertex, Polyline,
};

fn main() {
    polyline_creation();
    vertex_operations();
    closed_open_behavior();
}

fn polyline_creation() {
    // Empty polyline
    let empty = Polyline::<f64>::new();
    assert_eq!(
        empty.vertex_count(),
        0,
        "Empty polyline should have 0 vertices"
    );
    assert!(
        !empty.is_closed(),
        "Empty polyline should be open by default"
    );

    // Polyline with capacity
    let with_capacity = Polyline::<f64>::with_capacity(5, true);
    assert_eq!(
        with_capacity.vertex_count(),
        0,
        "Polyline with capacity should start empty"
    );
    assert!(
        with_capacity.is_closed(),
        "Polyline should be closed when specified in constructor"
    );

    // From iterator
    let vertices = vec![
        PlineVertex::new(0.0, 0.0, 0.0),
        PlineVertex::new(10.0, 0.0, 0.5), // Arc segment
        PlineVertex::new(0.0, 10.0, 0.0),
    ];
    let from_iter = Polyline::from_iter(vertices.into_iter(), true);
    assert_eq!(
        from_iter.vertex_count(),
        3,
        "Polyline from iterator should have 3 vertices"
    );
    assert!(
        from_iter.is_closed(),
        "Polyline from iterator should be closed when specified"
    );
    assert_eq!(
        from_iter.at(1).bulge,
        0.5,
        "Second vertex should have bulge value of 0.5"
    );
}

fn vertex_operations() {
    let mut pline = Polyline::<f64>::new();

    // Add vertices
    pline.add(0.0, 0.0, 0.0);
    pline.add(10.0, 0.0, 0.0);
    pline.add(10.0, 10.0, 0.0);
    assert_eq!(
        pline.vertex_count(),
        3,
        "Should have 3 vertices after adding them"
    );

    // Insert vertex
    pline.insert(1, 5.0, 0.0, 0.2);
    assert_eq!(
        pline.vertex_count(),
        4,
        "Should have 4 vertices after inserting one"
    );
    assert_eq!(pline.at(1).x, 5.0, "Inserted vertex should be at x=5.0");
    assert_eq!(
        pline.at(1).bulge,
        0.2,
        "Inserted vertex should have bulge=0.2"
    );

    // Add vertex with bulge
    pline.add_vertex(PlineVertex::new(0.0, 10.0, 0.5));
    assert_eq!(
        pline.vertex_count(),
        5,
        "Should have 5 vertices after adding vertex with bulge"
    );
    assert_eq!(pline.at(4).bulge, 0.5, "Last vertex should have bulge=0.5");

    // Remove operations
    let removed = pline.remove_last();
    assert_eq!(
        removed.bulge, 0.5,
        "Removed vertex should have the expected bulge value"
    );
    assert_eq!(
        pline.vertex_count(),
        4,
        "Should have 4 vertices after removing last"
    );

    let removed_at_index = pline.remove(1);
    assert_eq!(
        removed_at_index.x, 5.0,
        "Removed vertex at index 1 should have x=5.0"
    );
    assert_eq!(
        pline.vertex_count(),
        3,
        "Should have 3 vertices after removing at index"
    );

    // Vertex access and modification
    assert_eq!(
        pline.get(0).expect("index 0 should exist").x,
        0.0,
        "First vertex should have x=0.0"
    );
    assert!(
        pline.get(10).is_none(),
        "Out-of-bounds index should return None"
    );

    pline.set_vertex(0, PlineVertex::new(1.0, 1.0, 0.1));
    assert_eq!(pline.at(0).x, 1.0, "Modified vertex should have x=1.0");
    assert_eq!(
        pline.at(0).bulge,
        0.1,
        "Modified vertex should have bulge=0.1"
    );
}

fn closed_open_behavior() {
    let mut square = Polyline::new();
    square.add(0.0, 0.0, 0.0);
    square.add(10.0, 0.0, 0.0);
    square.add(10.0, 10.0, 0.0);
    square.add(0.0, 10.0, 0.0);

    // Test initial state
    assert!(!square.is_closed(), "Square should be open initially");

    // Set as closed and test segment count difference
    square.set_is_closed(true);
    assert!(
        square.is_closed(),
        "Square should be closed after setting is_closed to true"
    );
    let closed_segments = square.iter_segments().count();

    square.set_is_closed(false);
    let open_segments = square.iter_segments().count();

    // Closed polyline has one more segment (connecting last to first)
    assert_eq!(closed_segments, 4, "Closed square should have 4 segments");
    assert_eq!(open_segments, 3, "Open square should have 3 segments");

    // Test iteration behavior
    square.set_is_closed(true);
    let vertex_count = square.iter_vertexes().count();
    assert_eq!(
        vertex_count, 4,
        "Closed square should iterate over 4 vertices"
    );
}
