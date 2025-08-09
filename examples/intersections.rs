use cavalier_contours::{
    core::{traits::FuzzyEq, Control},
    polyline::{
        FindIntersectsOptions, PlineIntersect, PlineIntersectVisitContext, PlineIntersectVisitor,
        PlineSegIntr, PlineSelfIntersectOptions, PlineSource, PlineSourceMut, Polyline,
        TwoPlinesIntersectVisitor,
    },
};

fn main() {
    polyline_self_intersections();
    polyline_intersections();
}

fn polyline_self_intersections() {
    println!("Testing self-intersection detection...");

    // Create a polyline without self-intersections
    let mut simple_rect = Polyline::<f64>::new();
    simple_rect.add(0.0, 0.0, 0.0);
    simple_rect.add(10.0, 0.0, 0.0);
    simple_rect.add(10.0, 10.0, 0.0);
    simple_rect.add(0.0, 10.0, 0.0);
    simple_rect.set_is_closed(true);

    let has_self_intersect = simple_rect.scan_for_self_intersect();
    assert!(
        !has_self_intersect,
        "Simple rectangle should not have self-intersections"
    );
    println!("Simple rectangle has self-intersections: {has_self_intersect}");

    // Create a polyline with self-intersections (figure-8 shape) using closure visitor
    let mut figure_eight = Polyline::<f64>::new();
    figure_eight.add(0.0, 0.0, 0.0);
    figure_eight.add(10.0, 10.0, 0.0);
    figure_eight.add(10.0, 0.0, 0.0);
    figure_eight.add(0.0, 10.0, 0.0);
    figure_eight.set_is_closed(true);

    let mut intersection_count = 0;
    let mut first_intersection: Option<PlineIntersect<f64>> = None;

    figure_eight.visit_self_intersects(&mut |intersection| {
        intersection_count += 1;
        if first_intersection.is_none() {
            first_intersection = Some(intersection);
        }
        println!("  Found intersection #{intersection_count}: {intersection:?}");
    });

    assert!(
        intersection_count > 0,
        "Figure-eight shape should have self-intersections"
    );
    println!("Figure-eight shape (closure visitor): found {intersection_count} intersections");

    // Use custom options with visit_self_intersects_opt
    let mut options = PlineSelfIntersectOptions::new();
    options.pos_equal_eps = f64::fuzzy_epsilon() * 10.0;

    let mut opt_intersection_count = 0;
    figure_eight.visit_self_intersects_opt(
        &mut |intersection| {
            opt_intersection_count += 1;
            println!("  Custom options intersection #{opt_intersection_count}: {intersection:?}");
        },
        &options,
    );

    assert!(
        opt_intersection_count > 0,
        "Figure-eight should still have self-intersections with custom options"
    );
    println!("Figure-eight with custom options: found {opt_intersection_count} intersections");

    // Create a more complex self-intersecting shape using custom visitor
    let mut complex_shape = Polyline::<f64>::new();
    complex_shape.add(0.0, 0.0, 0.0);
    complex_shape.add(20.0, 0.0, 0.0);
    complex_shape.add(20.0, 10.0, 0.0);
    complex_shape.add(5.0, 10.0, 0.0);
    complex_shape.add(5.0, 20.0, 0.0);
    complex_shape.add(15.0, 20.0, 0.0);
    complex_shape.add(15.0, 5.0, 0.0);
    complex_shape.add(0.0, 5.0, 0.0);
    complex_shape.set_is_closed(true);

    let mut visitor = IntersectionCounter::new();
    complex_shape.visit_self_intersects(&mut visitor);

    assert!(
        visitor.count > 0,
        "Complex shape should have self-intersections"
    );
    println!(
        "Complex shape (custom visitor): found {} intersections",
        visitor.count
    );
    println!(
        "  Basic intersections: {}",
        visitor.basic_intersections.len()
    );
    println!(
        "  Overlapping intersections: {}",
        visitor.overlapping_intersections.len()
    );

    println!("Self-intersection detection completed successfully!\n");
}

fn polyline_intersections() {
    println!("Testing polyline-to-polyline intersections...");
    // Create two intersecting rectangles
    let mut rect1 = Polyline::<f64>::new();
    rect1.add(0.0, 0.0, 0.0);
    rect1.add(10.0, 0.0, 0.0);
    rect1.add(10.0, 10.0, 0.0);
    rect1.add(0.0, 10.0, 0.0);
    rect1.set_is_closed(true);

    let mut rect2 = Polyline::<f64>::new();
    rect2.add(5.0, 5.0, 0.0);
    rect2.add(15.0, 5.0, 0.0);
    rect2.add(15.0, 15.0, 0.0);
    rect2.add(5.0, 15.0, 0.0);
    rect2.set_is_closed(true);

    // Use visit_intersects with closure for detailed intersection processing
    let mut basic_count = 0;
    let mut overlapping_count = 0;
    let mut intersection_points = Vec::new();

    rect1.visit_intersects(&rect2, &mut |seg_intr, ctx1: &PlineIntersectVisitContext<f64>, ctx2: &PlineIntersectVisitContext<f64>| {
        match seg_intr {
            PlineSegIntr::NoIntersect => {
                // No intersection between these segments
            }
            PlineSegIntr::TangentIntersect { point } => {
                basic_count += 1;
                intersection_points.push((point.x, point.y));
                println!("  Tangent intersection #{basic_count}: ({:.2}, {:.2}) at segments ({}, {})", 
                         point.x, point.y, ctx1.vertex_index, ctx2.vertex_index);
                // Verify intersection points are within reasonable bounds
                assert!(point.x >= 0.0 && point.x <= 15.0, "Intersection X coordinate should be within bounds");
                assert!(point.y >= 0.0 && point.y <= 15.0, "Intersection Y coordinate should be within bounds");
            }
            PlineSegIntr::OneIntersect { point } => {
                basic_count += 1;
                intersection_points.push((point.x, point.y));
                println!("  Basic intersection #{basic_count}: ({:.2}, {:.2}) at segments ({}, {})", 
                         point.x, point.y, ctx1.vertex_index, ctx2.vertex_index);
                // Verify intersection points are within reasonable bounds
                assert!(point.x >= 0.0 && point.x <= 15.0, "Intersection X coordinate should be within bounds");
                assert!(point.y >= 0.0 && point.y <= 15.0, "Intersection Y coordinate should be within bounds");
            }
            PlineSegIntr::TwoIntersects { point1, point2 } => {
                basic_count += 2;
                intersection_points.push((point1.x, point1.y));
                intersection_points.push((point2.x, point2.y));
                println!("  Two intersections #{}: ({:.2}, {:.2}) and ({:.2}, {:.2}) at segments ({}, {})", 
                         basic_count - 1, point1.x, point1.y, point2.x, point2.y, ctx1.vertex_index, ctx2.vertex_index);
            }
            PlineSegIntr::OverlappingLines { point1, point2 } => {
                overlapping_count += 1;
                println!("  Overlapping line #{overlapping_count}: ({:.2}, {:.2}) to ({:.2}, {:.2}) at segments ({}, {})", 
                         point1.x, point1.y, point2.x, point2.y, ctx1.vertex_index, ctx2.vertex_index);
            }
            PlineSegIntr::OverlappingArcs { point1, point2 } => {
                overlapping_count += 1;
                println!("  Overlapping arc #{overlapping_count}: ({:.2}, {:.2}) to ({:.2}, {:.2}) at segments ({}, {})", 
                         point1.x, point1.y, point2.x, point2.y, ctx1.vertex_index, ctx2.vertex_index);
            }
        }
    });

    // Two overlapping rectangles should have exactly 2 intersection points
    assert_eq!(
        basic_count, 2,
        "Expected exactly 2 basic intersections between overlapping rectangles"
    );
    assert_eq!(
        overlapping_count, 0,
        "Expected no overlapping intersections for basic rectangle intersection"
    );
    println!("Intersecting rectangles (closure visitor):");
    println!("  Basic intersections: {basic_count}");
    println!("  Overlapping intersections: {overlapping_count}");

    // Non-intersecting polylines
    let mut rect3 = Polyline::<f64>::new();
    rect3.add(20.0, 20.0, 0.0);
    rect3.add(30.0, 20.0, 0.0);
    rect3.add(30.0, 30.0, 0.0);
    rect3.add(20.0, 30.0, 0.0);
    rect3.set_is_closed(true);

    let no_intersections = rect1.find_intersects(&rect3);
    assert_eq!(
        no_intersections.basic_intersects.len(),
        0,
        "Non-intersecting rectangles should have 0 basic intersections"
    );
    assert_eq!(
        no_intersections.overlapping_intersects.len(),
        0,
        "Non-intersecting rectangles should have 0 overlapping intersections"
    );
    println!("\nNon-intersecting rectangles:");
    println!(
        "  Basic intersections: {} (expected: 0)",
        no_intersections.basic_intersects.len()
    );
    println!(
        "  Overlapping intersections: {} (expected: 0)",
        no_intersections.overlapping_intersects.len()
    );

    // Use custom options for intersection finding
    let mut options = FindIntersectsOptions::new();
    options.pos_equal_eps = f64::fuzzy_epsilon() * 10.0;

    let intersections_opt = rect1.find_intersects_opt(&rect2, &options);
    // Custom options should still find the same intersections
    assert_eq!(
        intersections_opt.basic_intersects.len(),
        2,
        "Custom options should still find 2 basic intersections"
    );
    println!("\nIntersections with custom options:");
    println!(
        "  Basic intersections: {}",
        intersections_opt.basic_intersects.len()
    );
    println!(
        "  Overlapping intersections: {}",
        intersections_opt.overlapping_intersects.len()
    );

    // Intersections with arc segments using custom visitor
    let mut arc_line1 = Polyline::<f64>::new();
    arc_line1.add(0.0, 0.0, 0.5); // Arc bulge
    arc_line1.add(10.0, 0.0, 0.0);
    arc_line1.add(10.0, 5.0, 0.0);
    arc_line1.add(0.0, 5.0, 0.0);
    arc_line1.set_is_closed(true);

    let mut straight_line = Polyline::<f64>::new();
    straight_line.add(5.0, -2.0, 0.0);
    straight_line.add(5.0, 7.0, 0.0);

    let mut visitor = DetailedIntersectionVisitor::new();
    arc_line1.visit_intersects(&straight_line, &mut visitor);

    // Arc with vertical line should have at least 1 intersection
    assert!(
        !visitor.intersections.is_empty(),
        "Arc-line intersection should have at least 1 basic intersection"
    );
    println!("\nArc-line intersections (custom visitor):");
    println!(
        "  Total intersections found: {}",
        visitor.intersections.len()
    );

    for (i, detail) in visitor.intersections.iter().enumerate() {
        println!("  Intersection {i}: {detail}");
        // Vertical line is at x=5, so intersection should be close to x=5
        if let Some((x, _)) = detail.point() {
            assert!(x.fuzzy_eq(5.0), "Arc-line intersection should be near x=5");
        }
    }

    // Tangent intersections (touching but not crossing)
    let mut circle1 = Polyline::<f64>::new();
    circle1.add(0.0, 0.0, 1.0); // Semi-circle
    circle1.add(10.0, 0.0, 1.0); // Complete circle with two semi-circle segments
    circle1.set_is_closed(true);

    let mut circle2 = Polyline::<f64>::new();
    circle2.add(10.0, 0.0, 1.0); // Semi-circle touching the first one
    circle2.add(20.0, 0.0, 1.0); // Complete circle
    circle2.set_is_closed(true);

    let tangent_intersections = circle1.find_intersects(&circle2);
    println!("\nTangent circles:");
    println!(
        "  Basic intersections: {}",
        tangent_intersections.basic_intersects.len()
    );
    println!(
        "  Overlapping intersections: {}",
        tangent_intersections.overlapping_intersects.len()
    );

    // Overlapping segments
    let mut line1 = Polyline::<f64>::new();
    line1.add(0.0, 0.0, 0.0);
    line1.add(10.0, 0.0, 0.0);
    line1.add(10.0, 10.0, 0.0);

    let mut line2 = Polyline::<f64>::new();
    line2.add(5.0, 0.0, 0.0);
    line2.add(15.0, 0.0, 0.0);
    line2.add(15.0, 5.0, 0.0);

    let overlap_intersections = line1.find_intersects(&line2);
    // Overlapping line segments should have overlapping intersections
    assert!(
        !overlap_intersections.overlapping_intersects.is_empty()
            || !overlap_intersections.basic_intersects.is_empty(),
        "Overlapping line segments should have at least one intersection"
    );
    println!("\nOverlapping line segments:");
    println!(
        "  Basic intersections: {}",
        overlap_intersections.basic_intersects.len()
    );
    println!(
        "  Overlapping intersections: {}",
        overlap_intersections.overlapping_intersects.len()
    );

    for (i, overlap) in overlap_intersections
        .overlapping_intersects
        .iter()
        .enumerate()
    {
        println!(
            "  Overlap {}: ({:.2}, {:.2}) to ({:.2}, {:.2})",
            i, overlap.point1.x, overlap.point1.y, overlap.point2.x, overlap.point2.y
        );
        // Overlapping segments should be on the same horizontal line (y=0)
        assert!(
            overlap.point1.y.fuzzy_eq_zero(),
            "Overlapping segment should be on y=0 line"
        );
        assert!(
            overlap.point2.y.fuzzy_eq_zero(),
            "Overlapping segment should be on y=0 line"
        );
        // Points should be within the expected x range [5, 10]
        assert!(
            overlap.point1.x >= 5.0 && overlap.point1.x <= 10.0,
            "Overlap point1 x should be in range [5,10]"
        );
        assert!(
            overlap.point2.x >= 5.0 && overlap.point2.x <= 10.0,
            "Overlap point2 x should be in range [5,10]"
        );
    }

    println!("\nPolyline intersection tests completed successfully!\n");
}

// Custom visitor for detailed intersection information with context
#[derive(Debug)]
enum IntersectionDetail {
    NoIntersection,
    Tangent {
        point: (f64, f64),
        seg1: usize,
        seg2: usize,
    },
    Basic {
        point: (f64, f64),
        seg1: usize,
        seg2: usize,
    },
    Double {
        point1: (f64, f64),
        point2: (f64, f64),
        seg1: usize,
        seg2: usize,
    },
    OverlappingLine {
        start: (f64, f64),
        end: (f64, f64),
        seg1: usize,
        seg2: usize,
    },
    OverlappingArc {
        start: (f64, f64),
        end: (f64, f64),
        seg1: usize,
        seg2: usize,
    },
}

impl std::fmt::Display for IntersectionDetail {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            IntersectionDetail::NoIntersection => write!(f, "No intersection"),
            IntersectionDetail::Tangent { point, seg1, seg2 } => write!(
                f,
                "Tangent at ({:.2}, {:.2}) between segments {} and {}",
                point.0, point.1, seg1, seg2
            ),
            IntersectionDetail::Basic { point, seg1, seg2 } => write!(
                f,
                "Basic at ({:.2}, {:.2}) between segments {} and {}",
                point.0, point.1, seg1, seg2
            ),
            IntersectionDetail::Double {
                point1,
                point2,
                seg1,
                seg2,
            } => write!(
                f,
                "Double at ({:.2}, {:.2}) and ({:.2}, {:.2}) between segments {} and {}",
                point1.0, point1.1, point2.0, point2.1, seg1, seg2
            ),
            IntersectionDetail::OverlappingLine {
                start,
                end,
                seg1,
                seg2,
            } => write!(
                f,
                "Overlapping line from ({:.2}, {:.2}) to ({:.2}, {:.2}) between segments {} and {}",
                start.0, start.1, end.0, end.1, seg1, seg2
            ),
            IntersectionDetail::OverlappingArc {
                start,
                end,
                seg1,
                seg2,
            } => write!(
                f,
                "Overlapping arc from ({:.2}, {:.2}) to ({:.2}, {:.2}) between segments {} and {}",
                start.0, start.1, end.0, end.1, seg1, seg2
            ),
        }
    }
}

impl IntersectionDetail {
    fn point(&self) -> Option<(f64, f64)> {
        match self {
            IntersectionDetail::Tangent { point, .. } => Some(*point),
            IntersectionDetail::Basic { point, .. } => Some(*point),
            IntersectionDetail::Double { point1, .. } => Some(*point1),
            _ => None,
        }
    }
}

struct DetailedIntersectionVisitor {
    intersections: Vec<IntersectionDetail>,
}

impl DetailedIntersectionVisitor {
    fn new() -> Self {
        Self {
            intersections: Vec::new(),
        }
    }
}

impl TwoPlinesIntersectVisitor<f64, Control<()>> for DetailedIntersectionVisitor {
    fn visit(
        &mut self,
        intersect: PlineSegIntr<f64>,
        pline1_context: &PlineIntersectVisitContext<f64>,
        pline2_context: &PlineIntersectVisitContext<f64>,
    ) -> Control<()> {
        let detail = match intersect {
            PlineSegIntr::NoIntersect => IntersectionDetail::NoIntersection,
            PlineSegIntr::TangentIntersect { point } => IntersectionDetail::Tangent {
                point: (point.x, point.y),
                seg1: pline1_context.vertex_index,
                seg2: pline2_context.vertex_index,
            },
            PlineSegIntr::OneIntersect { point } => IntersectionDetail::Basic {
                point: (point.x, point.y),
                seg1: pline1_context.vertex_index,
                seg2: pline2_context.vertex_index,
            },
            PlineSegIntr::TwoIntersects { point1, point2 } => IntersectionDetail::Double {
                point1: (point1.x, point1.y),
                point2: (point2.x, point2.y),
                seg1: pline1_context.vertex_index,
                seg2: pline2_context.vertex_index,
            },
            PlineSegIntr::OverlappingLines { point1, point2 } => {
                IntersectionDetail::OverlappingLine {
                    start: (point1.x, point1.y),
                    end: (point2.x, point2.y),
                    seg1: pline1_context.vertex_index,
                    seg2: pline2_context.vertex_index,
                }
            }
            PlineSegIntr::OverlappingArcs { point1, point2 } => {
                IntersectionDetail::OverlappingArc {
                    start: (point1.x, point1.y),
                    end: (point2.x, point2.y),
                    seg1: pline1_context.vertex_index,
                    seg2: pline2_context.vertex_index,
                }
            }
        };

        // Only store actual intersections, not NoIntersection
        if !matches!(detail, IntersectionDetail::NoIntersection) {
            self.intersections.push(detail);
        }

        Control::Continue
    }
}

// Custom visitor that counts intersections and stores details
struct IntersectionCounter {
    count: usize,
    basic_intersections: Vec<(f64, f64)>, // Store intersection points
    overlapping_intersections: Vec<(f64, f64, f64, f64)>, // Store overlap segments
}

impl IntersectionCounter {
    fn new() -> Self {
        Self {
            count: 0,
            basic_intersections: Vec::new(),
            overlapping_intersections: Vec::new(),
        }
    }
}

impl PlineIntersectVisitor<f64, Control<()>> for IntersectionCounter {
    fn visit_basic_intr(
        &mut self,
        intr: cavalier_contours::polyline::PlineBasicIntersect<f64>,
    ) -> Control<()> {
        self.count += 1;
        self.basic_intersections.push((intr.point.x, intr.point.y));
        println!(
            "  Basic intersection #{}: ({:.2}, {:.2}) at segments ({}, {})",
            self.count, intr.point.x, intr.point.y, intr.start_index1, intr.start_index2
        );
        Control::Continue
    }

    fn visit_overlapping_intr(
        &mut self,
        intr: cavalier_contours::polyline::PlineOverlappingIntersect<f64>,
    ) -> Control<()> {
        self.count += 1;
        self.overlapping_intersections.push((
            intr.point1.x,
            intr.point1.y,
            intr.point2.x,
            intr.point2.y,
        ));
        println!(
            "  Overlapping intersection #{}: ({:.2}, {:.2}) to ({:.2}, {:.2}) at segments ({}, {})",
            self.count,
            intr.point1.x,
            intr.point1.y,
            intr.point2.x,
            intr.point2.y,
            intr.start_index1,
            intr.start_index2
        );
        Control::Continue
    }
}
