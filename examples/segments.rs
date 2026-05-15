use cavalier_contours::static_aabb2d_index::AABB;
use cavalier_contours::{
    core::{math::Vector2, traits::FuzzyEq},
    polyline::{
        pline_seg_intr, seg_arc_radius_and_center, seg_bounding_box, seg_closest_point, seg_length,
        seg_midpoint, seg_split_at_point, seg_tangent_vector, PlineSegIntr, PlineVertex,
    },
};

// Common test values
const SEMICIRCLE_BULGE: f64 = 1.0;
const QUARTER_BULGE: f64 = 0.5;

fn main() {
    segment_properties();
    segment_intersections();
    segment_closest_points();
    arc_segments();
    segment_utilities();
}

fn segment_properties() {
    println!("Testing basic segment properties...");

    // Line segment
    let line_seg = (
        PlineVertex::new(0.0, 0.0, 0.0),
        PlineVertex::new(10.0, 0.0, 0.0),
    );
    print_segment_info(line_seg.0, line_seg.1, "Line segment from (0,0) to (10,0)");

    // Arc segment (semicircle)
    let arc_seg = (
        PlineVertex::new(0.0, 0.0, SEMICIRCLE_BULGE),
        PlineVertex::new(10.0, 0.0, 0.0),
    );
    print_segment_info(arc_seg.0, arc_seg.1, "Arc segment (semicircle)");

    println!("Segment properties tests completed successfully!\n");
}

fn segment_intersections() {
    println!("Testing segment intersections...");

    // Test cases: (segment1, segment2, description)
    let test_cases = [
        (
            (
                PlineVertex::new(0.0, 0.0, 0.0),
                PlineVertex::new(10.0, 10.0, 0.0),
            ),
            (
                PlineVertex::new(0.0, 10.0, 0.0),
                PlineVertex::new(10.0, 0.0, 0.0),
            ),
            "Crossing lines",
        ),
        (
            (
                PlineVertex::new(0.0, 0.0, 0.0),
                PlineVertex::new(10.0, 0.0, 0.0),
            ),
            (
                PlineVertex::new(0.0, 5.0, 0.0),
                PlineVertex::new(10.0, 5.0, 0.0),
            ),
            "Parallel lines",
        ),
        (
            (
                PlineVertex::new(0.0, 0.0, SEMICIRCLE_BULGE),
                PlineVertex::new(10.0, 0.0, 0.0),
            ),
            (
                PlineVertex::new(5.0, -2.0, 0.0),
                PlineVertex::new(5.0, 7.0, 0.0),
            ),
            "Arc-line intersection",
        ),
    ];

    for (seg1, seg2, description) in &test_cases {
        println!("\n{description}:");
        let result = pline_seg_intr(seg1.0, seg1.1, seg2.0, seg2.1, f64::fuzzy_epsilon());
        print_intersection_result(result);
    }

    println!("Segment intersection tests completed successfully!\n");
}

fn segment_closest_points() {
    println!("Testing segment closest point calculations...");

    fn test_closest_point(
        seg: (PlineVertex<f64>, PlineVertex<f64>),
        point: Vector2<f64>,
        desc: &str,
    ) {
        println!("\n{desc}");
        print_point("Test point", point);
        let closest = seg_closest_point(seg.0, seg.1, point, f64::fuzzy_epsilon());
        print_point("Closest point", closest);
        println!("  Distance: {:.2}", (closest - point).length());
    }

    let line_seg = (
        PlineVertex::new(0.0, 0.0, 0.0),
        PlineVertex::new(10.0, 0.0, 0.0),
    );
    let arc_seg = (
        PlineVertex::new(0.0, 0.0, SEMICIRCLE_BULGE),
        PlineVertex::new(10.0, 0.0, 0.0),
    );

    test_closest_point(
        line_seg,
        Vector2::new(5.0, 3.0),
        "Line segment - point above",
    );
    test_closest_point(
        line_seg,
        Vector2::new(12.0, 3.0),
        "Line segment - point beyond end",
    );
    test_closest_point(
        arc_seg,
        Vector2::new(5.0, 2.0),
        "Arc segment - point outside",
    );
    test_closest_point(
        arc_seg,
        Vector2::new(5.0, 1.0),
        "Arc segment - point inside",
    );
    test_closest_point(
        line_seg,
        Vector2::new(0.0, 0.0),
        "Line segment - point at vertex",
    );

    println!("Segment closest point tests completed successfully!\n");
}

fn arc_segments() {
    println!("Testing arc segment operations...");

    let arc_types = [
        (QUARTER_BULGE, "Quarter circle"),
        (SEMICIRCLE_BULGE, "Semi-circle"),
        (-QUARTER_BULGE, "Clockwise quarter circle"),
        (-SEMICIRCLE_BULGE, "Clockwise semi-circle"),
    ];

    for (bulge, description) in &arc_types {
        let arc_seg = (
            PlineVertex::new(0.0, 0.0, *bulge),
            PlineVertex::new(10.0, 0.0, 0.0),
        );
        print_segment_info(
            arc_seg.0,
            arc_seg.1,
            &format!("{description} (bulge: {bulge:.1})"),
        );
    }

    println!("Arc segment tests completed successfully!\n");
}

fn segment_utilities() {
    println!("Testing segment utilities...");

    fn print_split_result(result: &cavalier_contours::polyline::SplitResult<f64>, desc: &str) {
        println!("\n{desc}");
        println!(
            "  Updated start: ({:.1}, {:.1}), bulge: {:.2}",
            result.updated_start.x, result.updated_start.y, result.updated_start.bulge
        );
        println!(
            "  Split vertex: ({:.1}, {:.1}), bulge: {:.2}",
            result.split_vertex.x, result.split_vertex.y, result.split_vertex.bulge
        );
    }

    // Segment splitting examples
    let line_seg = (
        PlineVertex::new(0.0, 0.0, 0.0),
        PlineVertex::new(10.0, 10.0, 0.0),
    );
    let line_split = seg_split_at_point(
        line_seg.0,
        line_seg.1,
        Vector2::new(3.0, 3.0),
        f64::fuzzy_epsilon(),
    );
    print_split_result(&line_split, "Line segment split at (3,3)");

    let arc_seg = (
        PlineVertex::new(0.0, 0.0, SEMICIRCLE_BULGE),
        PlineVertex::new(10.0, 0.0, 0.0),
    );
    let arc_split = seg_split_at_point(
        arc_seg.0,
        arc_seg.1,
        Vector2::new(5.0, 5.0),
        f64::fuzzy_epsilon(),
    );
    print_split_result(&arc_split, "Arc segment split at (5,5)");

    // Property comparison
    println!("\nLine vs Arc comparison:");
    let line = (
        PlineVertex::new(0.0, 0.0, 0.0),
        PlineVertex::new(10.0, 0.0, 0.0),
    );
    let arc = (
        PlineVertex::new(0.0, 0.0, SEMICIRCLE_BULGE),
        PlineVertex::new(10.0, 0.0, 0.0),
    );

    println!(
        "  Line length: {:.2}, Arc length: {:.2}",
        seg_length(line.0, line.1),
        seg_length(arc.0, arc.1)
    );

    // Tangent vectors along arc
    println!("\nTangent vectors along semicircle:");
    let points = [arc.0.pos(), Vector2::new(5.0, 5.0), arc.1.pos()];
    let labels = ["Start", "Top", "End"];

    for (point, label) in points.iter().zip(labels.iter()) {
        let tangent = seg_tangent_vector(arc.0, arc.1, *point);
        println!(
            "  {}: ({:.1}, {:.1}) -> Tangent: ({:.2}, {:.2})",
            label, point.x, point.y, tangent.x, tangent.y
        );
    }

    println!("Segment utilities tests completed successfully!\n");
}

fn print_point(label: &str, point: Vector2<f64>) {
    println!("  {}: ({:.2}, {:.2})", label, point.x, point.y);
}

fn print_bbox(bbox: &AABB<f64>) {
    println!(
        "  Bounding box: ({:.2}, {:.2}) to ({:.2}, {:.2})",
        bbox.min_x, bbox.min_y, bbox.max_x, bbox.max_y
    );
}

fn print_segment_info(start: PlineVertex<f64>, end: PlineVertex<f64>, name: &str) {
    println!("\n{name}:");
    println!("  Length: {:.2}", seg_length(start, end));
    let midpoint = seg_midpoint(start, end);
    print_point("Midpoint", midpoint);
    print_bbox(&seg_bounding_box(start, end));

    if start.bulge_is_zero() {
        print_point(
            "Tangent at start (not normalized)",
            seg_tangent_vector(start, end, start.pos()),
        );
    } else {
        let (radius, center) = seg_arc_radius_and_center(start, end);
        println!("  Arc radius: {radius:.2}");
        print_point("Arc center", center);
        print_point(
            "Tangent at start (not normalized)",
            seg_tangent_vector(start, end, start.pos()),
        );
        print_point(
            "Tangent at midpoint (not normalized)",
            seg_tangent_vector(start, end, midpoint),
        );
        print_point(
            "Tangent at endpoint (not normalized)",
            seg_tangent_vector(start, end, end.pos()),
        );
    }
}

fn print_intersection_result(result: PlineSegIntr<f64>) {
    match result {
        PlineSegIntr::NoIntersect => println!("  No intersection"),
        PlineSegIntr::TangentIntersect { point } => print_point("Tangent intersection", point),
        PlineSegIntr::OneIntersect { point } => print_point("Intersection", point),
        PlineSegIntr::TwoIntersects { point1, point2 } => {
            print_point("Intersection 1", point1);
            print_point("Intersection 2", point2);
        }
        PlineSegIntr::OverlappingLines { point1, point2 } => {
            println!("  Overlapping lines:");
            print_point("  From", point1);
            print_point("  To", point2);
        }
        PlineSegIntr::OverlappingArcs { point1, point2 } => {
            println!("  Overlapping arcs:");
            print_point("  From", point1);
            print_point("  To", point2);
        }
    }
}
