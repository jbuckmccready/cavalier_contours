use cavalier_contours::{
    core::traits::FuzzyEq,
    polyline::{PlineOrientation, PlineSource, PlineSourceMut, Polyline},
};

fn main() {
    extents();
    area_calculations();
    path_length();
    orientation();
}

fn extents() {
    println!("Testing bounding box extents...");

    // Rectangle extents
    let mut rect = Polyline::new();
    rect.add(0.0, 0.0, 0.0);
    rect.add(10.0, 0.0, 0.0);
    rect.add(10.0, 10.0, 0.0);
    rect.add(0.0, 10.0, 0.0);
    rect.set_is_closed(true);

    let extents = rect
        .extents()
        .expect("Rectangle polyline should not be empty");
    assert_eq!(extents.min_x, 0.0, "Rectangle min_x should be 0.0");
    assert_eq!(extents.min_y, 0.0, "Rectangle min_y should be 0.0");
    assert_eq!(extents.max_x, 10.0, "Rectangle max_x should be 10.0");
    assert_eq!(extents.max_y, 10.0, "Rectangle max_y should be 10.0");
    println!(
        "Rectangle extents: min({}, {}) to max({}, {})",
        extents.min_x, extents.min_y, extents.max_x, extents.max_y
    );

    // Circle extents using bulge values
    // Create a circle using 4 quarter-circle arcs, radius 5, centered at origin
    let quarter_circle_bulge = (std::f64::consts::PI / 8.0).tan();
    let mut circle = Polyline::new();
    circle.add(5.0, 0.0, quarter_circle_bulge); // Quarter circle to (0,5)
    circle.add(0.0, 5.0, quarter_circle_bulge); // Quarter circle to (-5,0)
    circle.add(-5.0, 0.0, quarter_circle_bulge); // Quarter circle to (0,-5)
    circle.add(0.0, -5.0, quarter_circle_bulge); // Quarter circle back to (5,0)
    circle.set_is_closed(true);

    let circle_extents = circle.extents().expect("Circle should not be empty");
    assert!(
        circle_extents.min_x.fuzzy_eq(-5.0),
        "Circle min_x should be -5.0"
    );
    assert!(
        circle_extents.min_y.fuzzy_eq(-5.0),
        "Circle min_y should be -5.0"
    );
    assert!(
        circle_extents.max_x.fuzzy_eq(5.0),
        "Circle max_x should be 5.0"
    );
    assert!(
        circle_extents.max_y.fuzzy_eq(5.0),
        "Circle max_y should be 5.0"
    );
    println!(
        "Circle extents: min({:.1}, {:.1}) to max({:.1}, {:.1})",
        circle_extents.min_x, circle_extents.min_y, circle_extents.max_x, circle_extents.max_y
    );

    // Empty polyline has no extents
    let empty = Polyline::<f64>::new();
    assert!(
        empty.extents().is_none(),
        "Empty polyline should have no extents"
    );
    println!("Empty polyline: no extents as expected");

    println!("Extents calculations completed successfully!\n");
}

fn area_calculations() {
    println!("Testing area calculations...");

    let mut square = Polyline::new();
    square.add(0.0, 0.0, 0.0);
    square.add(10.0, 0.0, 0.0);
    square.add(10.0, 10.0, 0.0);
    square.add(0.0, 10.0, 0.0);
    square.set_is_closed(true);

    let area = square.area();
    assert!(area.fuzzy_eq(100.0), "Square area should be 100.0 (10x10)");
    println!("Square area: {area:.2} (10x10 = 100)");

    // Triangle area
    let mut triangle = Polyline::new();
    triangle.add(0.0, 0.0, 0.0);
    triangle.add(10.0, 0.0, 0.0);
    triangle.add(5.0, 10.0, 0.0);
    triangle.set_is_closed(true);

    let triangle_area = triangle.area();
    assert!(
        triangle_area.fuzzy_eq(50.0),
        "Triangle area should be 50.0 (base×height/2)"
    );
    println!("Triangle area: {triangle_area:.2} (base×height/2 = 10×10/2 = 50)");

    // Open polyline has zero area
    let mut open_line = Polyline::new();
    open_line.add(0.0, 0.0, 0.0);
    open_line.add(10.0, 0.0, 0.0);
    open_line.add(10.0, 10.0, 0.0);

    let open_area: f64 = open_line.area();
    assert!(
        open_area.fuzzy_eq_zero(),
        "Open polyline should have zero area"
    );
    println!("Open polyline area: {open_area:.2} (open shapes have zero area)");

    // Semi-circle area (approximate test)
    let mut arc_shape = Polyline::new();
    arc_shape.add(0.0, 0.0, 1.0); // Semi-circle
    arc_shape.add(10.0, 0.0, 0.0);
    arc_shape.set_is_closed(true);

    let arc_area = arc_shape.area();
    let expected_semicircle = std::f64::consts::PI * 25.0 / 2.0; // π * r² / 2, r = 5
    assert!(
        arc_area.fuzzy_eq_eps(expected_semicircle, 1.0),
        "Semi-circle area should match expected value within tolerance"
    );
    println!("Semi-circle area: {arc_area:.2} (expected: {expected_semicircle:.2})");

    // Full circle area using quarter-circle arcs
    let quarter_circle_bulge = (std::f64::consts::PI / 8.0).tan();
    let mut circle = Polyline::new();
    circle.add(5.0, 0.0, quarter_circle_bulge);
    circle.add(0.0, 5.0, quarter_circle_bulge);
    circle.add(-5.0, 0.0, quarter_circle_bulge);
    circle.add(0.0, -5.0, quarter_circle_bulge);
    circle.set_is_closed(true);

    let circle_area = circle.area();
    let expected_circle_area = std::f64::consts::PI * 25.0; // π * r², r = 5
    assert!(
        circle_area.fuzzy_eq(expected_circle_area),
        "Circle area should match π×r²"
    );
    println!("Circle area: {circle_area:.2} (expected: {expected_circle_area:.2})");

    println!("Area calculations completed successfully!\n");
}

fn path_length() {
    println!("Testing path length calculations...");

    // Two collinear segments of length 5 each
    let mut line = Polyline::new();
    line.add(0.0, 0.0, 0.0);
    line.add(3.0, 4.0, 0.0);
    line.add(6.0, 8.0, 0.0);

    let length = line.path_length();
    assert!(
        length.fuzzy_eq(10.0),
        "Path length should be 10.0 (two segments of 5 each)"
    );
    println!("Two collinear segments: length = {length:.2} (two segments of 5 each)");

    let mut square = Polyline::new();
    square.add(0.0, 0.0, 0.0);
    square.add(10.0, 0.0, 0.0);
    square.add(10.0, 10.0, 0.0);
    square.add(0.0, 10.0, 0.0);
    square.set_is_closed(true);

    let perimeter = square.path_length();
    assert!(
        perimeter.fuzzy_eq(40.0),
        "Square perimeter should be 40.0 (4×10)"
    );
    println!("Square perimeter: {perimeter:.2} (4×10 = 40)");

    // Empty polyline
    let empty = Polyline::<f64>::new();
    let empty_length = empty.path_length();
    assert!(
        empty_length.fuzzy_eq_zero(),
        "Empty polyline should have zero length"
    );
    println!("Empty polyline: length = 0.00");

    // Arc segment (approximate test)
    let mut arc_shape = Polyline::new();
    arc_shape.add(0.0, 0.0, 1.0); // Semi-circle
    arc_shape.add(10.0, 0.0, 0.0);

    let arc_length = arc_shape.path_length();
    let expected_arc = std::f64::consts::PI * 5.0; // π * radius
    assert!(
        arc_length.fuzzy_eq_eps(expected_arc, 1.0),
        "Semi-circle arc length should match expected value within tolerance"
    );
    println!("Semi-circle arc length: {arc_length:.2} (expected: {expected_arc:.2})");

    // Full circle circumference using quarter-circle arcs
    let quarter_circle_bulge = (std::f64::consts::PI / 8.0).tan();
    let mut circle = Polyline::new();
    circle.add(5.0, 0.0, quarter_circle_bulge);
    circle.add(0.0, 5.0, quarter_circle_bulge);
    circle.add(-5.0, 0.0, quarter_circle_bulge);
    circle.add(0.0, -5.0, quarter_circle_bulge);
    circle.set_is_closed(true);

    let circle_circumference = circle.path_length();
    let expected_circumference = 2.0 * std::f64::consts::PI * 5.0; // 2π * r
    assert!(
        circle_circumference.fuzzy_eq(expected_circumference),
        "Circle circumference should match 2π×r"
    );
    println!(
        "Circle circumference: {circle_circumference:.2} (expected: {expected_circumference:.2})"
    );

    println!("Path length calculations completed successfully!\n");
}

fn orientation() {
    println!("Testing polyline orientation...");

    let mut ccw_square = Polyline::new();
    ccw_square.add(0.0, 0.0, 0.0);
    ccw_square.add(10.0, 0.0, 0.0);
    ccw_square.add(10.0, 10.0, 0.0);
    ccw_square.add(0.0, 10.0, 0.0);
    ccw_square.set_is_closed(true);

    assert_eq!(
        ccw_square.orientation(),
        PlineOrientation::CounterClockwise,
        "CCW square should have CounterClockwise orientation"
    );
    assert!(
        ccw_square.area() > 0.0,
        "CCW square should have positive area"
    );
    println!(
        "Counter-clockwise square: positive area = {:.2}",
        ccw_square.area()
    );

    // Clockwise square (reverse vertex order)
    let mut cw_square = Polyline::new();
    cw_square.add(0.0, 0.0, 0.0);
    cw_square.add(0.0, 10.0, 0.0);
    cw_square.add(10.0, 10.0, 0.0);
    cw_square.add(10.0, 0.0, 0.0);
    cw_square.set_is_closed(true);

    assert_eq!(
        cw_square.orientation(),
        PlineOrientation::Clockwise,
        "CW square should have Clockwise orientation"
    );
    assert!(
        cw_square.area() < 0.0,
        "CW square should have negative area"
    );
    println!("Clockwise square: negative area = {:.2}", cw_square.area());

    // Open polyline has "Open" orientation
    let mut open_line = Polyline::new();
    open_line.add(0.0, 0.0, 0.0);
    open_line.add(10.0, 0.0, 0.0);
    open_line.add(10.0, 10.0, 0.0);

    assert_eq!(
        open_line.orientation(),
        PlineOrientation::Open,
        "Open polyline should have Open orientation"
    );
    println!("Open polyline: orientation = Open");

    // Circle orientation using half-circle arcs
    // bulge = tan(π/4) = 1.0 for semicircle (180° arc)
    let semicircle_bulge = 1.0;
    let mut circle = Polyline::new();
    circle.add(5.0, 0.0, semicircle_bulge); // Semicircle from (5,0) to (-5,0)
    circle.add(-5.0, 0.0, semicircle_bulge); // Semicircle from (-5,0) back to (5,0)
    circle.set_is_closed(true);

    assert_eq!(
        circle.orientation(),
        PlineOrientation::CounterClockwise,
        "Circle should have CounterClockwise orientation"
    );
    assert!(circle.area() > 0.0, "Circle should have positive area");
    println!(
        "Circle orientation (2 semicircles): counter-clockwise with positive area = {:.2}",
        circle.area()
    );

    println!("Orientation tests completed successfully!\n");
}
