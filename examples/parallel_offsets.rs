use cavalier_contours::{
    core::traits::FuzzyEq,
    polyline::{PlineOffsetOptions, PlineSource, PlineSourceMut, Polyline},
    shape_algorithms::{Shape, ShapeOffsetOptions},
};

fn main() {
    single_polyline_offset();
    shape_offset();
}

fn single_polyline_offset() {
    println!("Testing single polyline parallel offsetting...");

    // Create a simple rectangle
    let mut rect = Polyline::<f64>::new();
    rect.add(0.0, 0.0, 0.0);
    rect.add(10.0, 0.0, 0.0);
    rect.add(10.0, 10.0, 0.0);
    rect.add(0.0, 10.0, 0.0);
    rect.set_is_closed(true);

    println!("Original rectangle:");
    println!("  Area: {:.2}", rect.area());
    println!("  Vertices: {}", rect.vertex_count());

    // Outward offset for CCW closed polyline uses NEGATIVE distance
    // (positive moves to the left of segment normal; CCW + positive => inward)
    let outward_offset = -2.0;
    let outward_results = rect.parallel_offset(outward_offset);

    println!("\nOutward offset by {:.2}:", outward_offset.abs());
    println!("  Number of result polylines: {}", outward_results.len());

    for (i, pline) in outward_results.iter().enumerate() {
        println!(
            "  Result {}: Area: {:.2}, Vertices: {}",
            i,
            pline.area(),
            pline.vertex_count()
        );
    }

    assert!(
        !outward_results.is_empty(),
        "Outward offset should produce at least one result polyline"
    );
    assert!(
        outward_results.iter().all(|p| p.is_closed()),
        "All outward offset results should be closed polylines"
    );

    let orig_ext = rect.extents().expect("rect has extents");
    let union_out_ext = outward_results
        .iter()
        .filter_map(|p| p.extents())
        .fold(None, |acc, e| match acc {
            None => Some(e),
            Some(mut u) => {
                u.min_x = u.min_x.min(e.min_x);
                u.min_y = u.min_y.min(e.min_y);
                u.max_x = u.max_x.max(e.max_x);
                u.max_y = u.max_y.max(e.max_y);
                Some(u)
            }
        })
        .expect("outward results have extents");

    assert!(
        union_out_ext.min_x <= orig_ext.min_x,
        "Outward offset should extend or maintain minimum x boundary"
    );
    assert!(
        union_out_ext.min_y <= orig_ext.min_y,
        "Outward offset should extend or maintain minimum y boundary"
    );
    assert!(
        union_out_ext.max_x >= orig_ext.max_x,
        "Outward offset should extend or maintain maximum x boundary"
    );
    assert!(
        union_out_ext.max_y >= orig_ext.max_y,
        "Outward offset should extend or maintain maximum y boundary"
    );

    // Inward offset for CCW closed polyline uses POSITIVE distance
    let inward_offset = 1.5;
    let inward_results = rect.parallel_offset(inward_offset);

    println!("\nInward offset by {inward_offset:.2}:");
    println!("  Number of result polylines: {}", inward_results.len());

    // Assert properties of inward offset results
    assert!(
        !inward_results.is_empty(),
        "Inward offset should produce at least one result polyline"
    );
    let total_in_area: f64 = inward_results.iter().map(|p| p.area().abs()).sum();
    assert!(
        total_in_area > 0.0 && total_in_area < rect.area().abs(),
        "Inward offset total area should be positive and less than original"
    );

    for (i, pline) in inward_results.iter().enumerate() {
        println!(
            "  Result {}: Area: {:.2}, Vertices: {}",
            i,
            pline.area(),
            pline.vertex_count()
        );
    }

    // Offset with custom options
    let mut options = PlineOffsetOptions::new();
    options.handle_self_intersects = true;
    options.pos_equal_eps = f64::fuzzy_epsilon() * 10.0;

    let custom_results = rect.parallel_offset_opt(1.0, &options);
    println!("\nOffset with custom options:");
    println!("  Number of result polylines: {}", custom_results.len());
    assert!(
        !custom_results.is_empty(),
        "Custom options offset should produce at least one result polyline"
    );

    // Shape with arc segments
    let mut arc_shape = Polyline::<f64>::new();
    arc_shape.add(0.0, 0.0, 0.5); // Arc bulge
    arc_shape.add(10.0, 0.0, 0.0);
    arc_shape.add(10.0, 10.0, -0.3); // Arc bulge (clockwise)
    arc_shape.add(0.0, 10.0, 0.0);
    arc_shape.set_is_closed(true);

    let arc_results = arc_shape.parallel_offset(1.0);
    println!("\nShape with arcs offset by 1.00:");
    println!("  Number of result polylines: {}", arc_results.len());
    assert!(
        !arc_results.is_empty(),
        "Arc shape offset should produce at least one result polyline"
    );
    for (i, pline) in arc_results.iter().enumerate() {
        println!(
            "  Result {}: Area: {:.2}, Vertices: {}",
            i,
            pline.area(),
            pline.vertex_count()
        );
    }

    // Open polyline offset
    let mut open_line = Polyline::<f64>::new();
    open_line.add(0.0, 0.0, 0.0);
    open_line.add(5.0, 0.0, 0.0);
    open_line.add(5.0, 5.0, 0.0);
    open_line.add(10.0, 5.0, 0.0);

    let open_results = open_line.parallel_offset(1.0);
    println!("\nOpen polyline offset by 1.00:");
    println!("  Number of result polylines: {}", open_results.len());
    assert!(
        !open_results.is_empty(),
        "Open polyline offset should produce at least one result polyline"
    );
    for (i, pline) in open_results.iter().enumerate() {
        println!(
            "  Result {}: Path length: {:.2}, Vertices: {}",
            i,
            pline.path_length(),
            pline.vertex_count()
        );
    }

    println!("Single polyline offset tests completed successfully!\n");
}

fn shape_offset() {
    // Create a shape with multiple polylines (rectangle with hole)
    let mut outer_rect = Polyline::<f64>::new();
    outer_rect.add(0.0, 0.0, 0.0);
    outer_rect.add(20.0, 0.0, 0.0);
    outer_rect.add(20.0, 20.0, 0.0);
    outer_rect.add(0.0, 20.0, 0.0);
    outer_rect.set_is_closed(true);

    // Inner hole (clockwise to create hole)
    let mut inner_rect = Polyline::<f64>::new();
    inner_rect.add(5.0, 5.0, 0.0);
    inner_rect.add(5.0, 15.0, 0.0);
    inner_rect.add(15.0, 15.0, 0.0);
    inner_rect.add(15.0, 5.0, 0.0);
    inner_rect.set_is_closed(true);

    // Create shape from polylines
    let polylines = vec![outer_rect, inner_rect];
    let shape = Shape::from_plines(polylines);

    println!("Original shape:");
    println!(
        "  CCW polylines (positive area): {}",
        shape.ccw_plines.len()
    );
    println!("  CW polylines (holes): {}", shape.cw_plines.len());

    // Calculate total (net) area and extents for original shape
    let total_area: f64 = shape
        .ccw_plines
        .iter()
        .map(|p| p.polyline.area().abs())
        .sum::<f64>()
        - shape
            .cw_plines
            .iter()
            .map(|p| p.polyline.area().abs())
            .sum::<f64>();
    println!("  Net area (ccw_plines - cw_plines): {total_area:.2}");

    // Note on sign convention for Shape::parallel_offset:
    // - Each loop is offset according to its orientation using the same rule as Polyline::parallel_offset
    // - CCW (positive area) + positive distance -> inward; negative -> outward
    // - CW (holes; negative area) + positive distance -> outward; negative -> inward
    // For a typical shape (islands CCW + holes CW), using a negative distance grows outer boundaries
    // and shrinks holes (increasing net area), while a positive distance shrinks outer boundaries
    // and grows holes (decreasing net area).
    let offset_options = ShapeOffsetOptions::new();
    let outward_shape = shape.parallel_offset(-2.0, offset_options.clone());

    println!("\nOutward offset by 2.00:");
    println!("  CCW polylines: {}", outward_shape.ccw_plines.len());
    println!("  CW polylines: {}", outward_shape.cw_plines.len());

    let outward_total_area: f64 = outward_shape
        .ccw_plines
        .iter()
        .map(|p| p.polyline.area().abs())
        .sum::<f64>()
        - outward_shape
            .cw_plines
            .iter()
            .map(|p| p.polyline.area().abs())
            .sum::<f64>();
    println!("  Net area (ccw_plines - cw_plines): {outward_total_area:.2}");

    // Inward offset (shrinks CCW islands and grows CW holes)
    let inward_shape = shape.parallel_offset(1.0, offset_options.clone());

    println!("\nInward offset by 1.00:");
    println!("  CCW polylines: {}", inward_shape.ccw_plines.len());
    println!("  CW polylines: {}", inward_shape.cw_plines.len());

    let inward_total_area: f64 = inward_shape
        .ccw_plines
        .iter()
        .map(|p| p.polyline.area().abs())
        .sum::<f64>()
        - inward_shape
            .cw_plines
            .iter()
            .map(|p| p.polyline.area().abs())
            .sum::<f64>();
    println!("  Net area (ccw_plines - cw_plines): {inward_total_area:.2}");

    // Create a more complex shape with multiple islands
    let mut island1 = Polyline::<f64>::new();
    island1.add(25.0, 5.0, 0.0);
    island1.add(30.0, 5.0, 0.0);
    island1.add(30.0, 10.0, 0.0);
    island1.add(25.0, 10.0, 0.0);
    island1.set_is_closed(true);

    let mut island2 = Polyline::<f64>::new();
    island2.add(25.0, 15.0, 0.0);
    island2.add(30.0, 15.0, 0.0);
    island2.add(30.0, 20.0, 0.0);
    island2.add(25.0, 20.0, 0.0);
    island2.set_is_closed(true);

    let complex_shape = Shape::from_plines(vec![
        // Main outer boundary
        {
            let mut outer = Polyline::<f64>::new();
            outer.add(0.0, 0.0, 0.0);
            outer.add(35.0, 0.0, 0.0);
            outer.add(35.0, 25.0, 0.0);
            outer.add(0.0, 25.0, 0.0);
            outer.set_is_closed(true);
            outer
        },
        // Hole
        {
            let mut hole = Polyline::<f64>::new();
            hole.add(5.0, 5.0, 0.0);
            hole.add(5.0, 20.0, 0.0);
            hole.add(15.0, 20.0, 0.0);
            hole.add(15.0, 5.0, 0.0);
            hole.set_is_closed(true);
            hole
        },
        island1,
        island2,
    ]);

    println!("\nComplex shape with islands:");
    println!("  CCW polylines: {}", complex_shape.ccw_plines.len());
    println!("  CW polylines: {}", complex_shape.cw_plines.len());

    // Compute original complex area for comparison
    let complex_area: f64 = complex_shape
        .ccw_plines
        .iter()
        .map(|p| p.polyline.area().abs())
        .sum::<f64>()
        - complex_shape
            .cw_plines
            .iter()
            .map(|p| p.polyline.area().abs())
            .sum::<f64>();

    let complex_offset = complex_shape.parallel_offset(1.0, offset_options);
    println!("\nComplex shape offset by 1.00:");
    println!("  CCW polylines: {}", complex_offset.ccw_plines.len());
    println!("  CW polylines: {}", complex_offset.cw_plines.len());

    // Display individual polyline details
    println!("\nDetailed results:");
    for (i, pline) in complex_offset.ccw_plines.iter().enumerate() {
        println!(
            "  CCW polyline {}: Area: {:.2}, Vertices: {}",
            i,
            pline.polyline.area(),
            pline.polyline.vertex_count()
        );
    }
    for (i, pline) in complex_offset.cw_plines.iter().enumerate() {
        println!(
            "  CW polyline {}: Area: {:.2}, Vertices: {}",
            i,
            pline.polyline.area(),
            pline.polyline.vertex_count()
        );
    }

    let complex_offset_area: f64 = complex_offset
        .ccw_plines
        .iter()
        .map(|p| p.polyline.area().abs())
        .sum::<f64>()
        - complex_offset
            .cw_plines
            .iter()
            .map(|p| p.polyline.area().abs())
            .sum::<f64>();
    println!(
        "  Net area (ccw_plines - cw_plines) changed from {:.2} to {:.2} (delta {:.2})",
        complex_area,
        complex_offset_area,
        complex_offset_area - complex_area
    );

    println!("Shape offset tests completed successfully!\n");
}
