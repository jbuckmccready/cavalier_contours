use cavalier_contours::{
    core::math::Vector2,
    core::traits::FuzzyEq,
    polyline::{PlineSource, PlineSourceMut, Polyline},
};

fn main() {
    winding_numbers();
    closest_points();
}

fn winding_numbers() {
    println!("Testing winding number calculations...");

    // Counter-clockwise square
    let mut square = Polyline::new();
    square.add(0.0, 0.0, 0.0);
    square.add(10.0, 0.0, 0.0);
    square.add(10.0, 10.0, 0.0);
    square.add(0.0, 10.0, 0.0);
    square.set_is_closed(true);

    let inside_point = Vector2::new(5.0, 5.0);
    let outside_point = Vector2::new(15.0, 5.0);

    assert_eq!(
        square.winding_number(inside_point),
        1,
        "Inside point should have winding number 1 for CCW square"
    );
    println!("Counter-clockwise square: inside point winding number = 1");

    assert_eq!(
        square.winding_number(outside_point),
        0,
        "Outside point should have winding number 0"
    );
    println!("Counter-clockwise square: outside point winding number = 0");

    // Clockwise square (invert direction)
    let cw_square = {
        square.invert_direction_mut();
        square
    };

    assert_eq!(
        cw_square.winding_number(inside_point),
        -1,
        "Inside point should have winding number -1 for CW square"
    );
    println!("Clockwise square: inside point winding number = -1");

    // Open polyline always returns 0
    let mut open_line = Polyline::new();
    open_line.add(0.0, 0.0, 0.0);
    open_line.add(10.0, 0.0, 0.0);
    open_line.add(10.0, 10.0, 0.0);

    assert_eq!(
        open_line.winding_number(inside_point),
        0,
        "Open polyline should always have winding number 0"
    );
    println!("Open polyline: winding number = 0 (as expected)");

    println!("Winding number tests completed successfully!\n");
}

fn closest_points() {
    println!("Testing closest point calculations...");

    let pos_eps = f64::fuzzy_epsilon();

    // Line segment test
    let mut line = Polyline::new();
    line.add(0.0, 0.0, 0.0);
    line.add(10.0, 0.0, 0.0);

    let test_point = Vector2::new(5.0, 3.0);
    let result = line
        .closest_point(test_point, pos_eps)
        .expect("Line should not be empty");
    assert_eq!(
        result.seg_start_index, 0,
        "Closest point should be on first segment"
    );
    assert!(
        result.seg_point.x.fuzzy_eq(5.0),
        "Closest point x-coordinate should be 5.0"
    );
    assert!(
        result.seg_point.y.fuzzy_eq_zero(),
        "Closest point y-coordinate should be 0.0"
    );
    assert!(
        result.distance.fuzzy_eq(3.0),
        "Distance to closest point should be 3.0"
    );
    println!("Line segment: closest point to (5.00,3.00) is (5.00,0.00) with distance 3.00");

    // Triangle test
    let mut triangle = Polyline::new();
    triangle.add(0.0, 0.0, 0.0);
    triangle.add(10.0, 0.0, 0.0);
    triangle.add(5.0, 10.0, 0.0);
    triangle.set_is_closed(true);

    let inside_point = Vector2::new(5.0, 2.0);
    let outside_point = Vector2::new(15.0, 5.0);

    let inside_result = triangle
        .closest_point(inside_point, pos_eps)
        .expect("Triangle should not be empty");
    let outside_result = triangle
        .closest_point(outside_point, pos_eps)
        .expect("Triangle should not be empty");

    assert!(
        inside_result.distance > 0.0,
        "Distance to inside point should be positive"
    );
    assert!(
        outside_result.distance > 0.0,
        "Distance to outside point should be positive"
    );
    println!("Triangle: found closest points for both inside and outside test points");

    // Empty polyline returns None
    let empty = Polyline::new();
    assert!(
        empty.closest_point(test_point, pos_eps).is_none(),
        "Empty polyline should return None for closest point"
    );
    println!("Empty polyline: correctly returns None for closest point");

    // Single vertex test
    let mut single_vertex = Polyline::new();
    single_vertex.add(5.0, 5.0, 0.0);

    let single_result = single_vertex
        .closest_point(Vector2::new(8.0, 9.0), pos_eps)
        .expect("Single vertex polyline should not be empty");
    assert_eq!(
        single_result.seg_point.x, 5.0,
        "Single vertex closest point x should be 5.0"
    );
    assert_eq!(
        single_result.seg_point.y, 5.0,
        "Single vertex closest point y should be 5.0"
    );
    assert!(
        single_result.distance.fuzzy_eq(5.0),
        "Distance to single vertex should be 5.0"
    );
    println!("Single vertex: closest point to (8.00,9.00) is (5.00,5.00) with distance 5.00");

    println!("Closest point tests completed successfully!\n");
}
