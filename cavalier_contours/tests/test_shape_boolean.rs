use cavalier_contours::core::{math::Vector2, traits::FuzzyEq};
use cavalier_contours::polyline::{
    BooleanOp, PlineOrientation, PlineSource, PlineSourceMut, Polyline,
};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};
use cavalier_contours::{assert_fuzzy_eq, pline_closed, pline_open};
use std::f64::consts::PI;

const SHAPE_TEST_EPS: f64 = 1e-7;

/// A small helper to verify bounding boxes.
fn assert_extents_fuzzy_eq(
    shape: &Shape<f64>,
    expected_min_x: f64,
    expected_min_y: f64,
    expected_max_x: f64,
    expected_max_y: f64,
    eps: f64,
) {
    let aabb = shape.plines_index.bounds().unwrap();
    assert!(
        aabb.min_x.fuzzy_eq_eps(expected_min_x, eps),
        "min_x => expected: {}, got: {}",
        expected_min_x,
        aabb.min_x
    );
    assert!(
        aabb.min_y.fuzzy_eq_eps(expected_min_y, eps),
        "min_y => expected: {}, got: {}",
        expected_min_y,
        aabb.min_y
    );
    assert!(
        aabb.max_x.fuzzy_eq_eps(expected_max_x, eps),
        "max_x => expected: {}, got: {}",
        expected_max_x,
        aabb.max_x
    );
    assert!(
        aabb.max_y.fuzzy_eq_eps(expected_max_y, eps),
        "max_y => expected: {}, got: {}",
        expected_max_y,
        aabb.max_y
    );
}

/// Create a simple rectangle polyline oriented along the XY axes.
fn create_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = Polyline::new_closed();
    pl.add(xmin, ymin, 0.0);
    pl.add(xmax, ymin, 0.0);
    pl.add(xmax, ymax, 0.0);
    pl.add(xmin, ymax, 0.0);
    pl
}

/// Create a circle-like shape from a small polyline (approx. circle).
fn create_approx_circle(center_x: f64, center_y: f64, radius: f64) -> Polyline<f64> {
    // Two vertices with bulge 1.0 encode a full circle as two half-circle arc segments.
    let mut pl = Polyline::new_closed();
    pl.add(center_x - radius, center_y, 1.0);
    pl.add(center_x + radius, center_y, 1.0);
    pl
}

fn create_cw_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = create_rectangle(xmin, ymin, xmax, ymax);
    pl.invert_direction_mut();
    pl
}

fn create_shape_rects(rects: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    Shape::from_plines(
        rects
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_rectangle(xmin, ymin, xmax, ymax)),
    )
}

fn create_donut(outer: (f64, f64, f64, f64), holes: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    let mut plines = vec![create_rectangle(outer.0, outer.1, outer.2, outer.3)];
    plines.extend(
        holes
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_cw_rectangle(xmin, ymin, xmax, ymax)),
    );
    Shape::from_plines(plines)
}

fn rotate_pline_start(pline: &Polyline<f64>, start_index: usize) -> Polyline<f64> {
    pline
        .rotate_start(start_index, pline.at(start_index).pos(), SHAPE_TEST_EPS)
        .expect("expected valid closed polyline rotation")
}

fn shape_signed_area(shape: &Shape<f64>) -> f64 {
    // Shape uses signed loop bins: CCW contributes positive area, CW contributes negative holes.
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.area())
        .chain(shape.cw_plines.iter().map(|ip| ip.polyline.area()))
        .sum()
}

fn shape_winding_number(shape: &Shape<f64>, point: Vector2<f64>) -> i32 {
    // Match Shape::boolean's non-zero winding semantics by summing every signed loop.
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.winding_number(point))
        .chain(
            shape
                .cw_plines
                .iter()
                .map(|ip| ip.polyline.winding_number(point)),
        )
        .sum()
}

fn shape_contains(shape: &Shape<f64>, x: f64, y: f64) -> bool {
    shape_winding_number(shape, Vector2::new(x, y)) != 0
}

fn assert_loop_valid(pline: &Polyline<f64>, expected_orientation: PlineOrientation) {
    assert!(pline.is_closed(), "shape loops must be closed: {:?}", pline);
    assert_eq!(
        pline.orientation(),
        expected_orientation,
        "loop orientation is in the wrong shape bin: {:?}",
        pline
    );
    assert!(
        pline.remove_repeat_pos(SHAPE_TEST_EPS).is_none(),
        "shape loop has repeat-position vertices: {:?}",
        pline
    );

    for v in pline.iter_vertexes() {
        assert!(v.x.is_finite(), "non-finite x coordinate in {:?}", pline);
        assert!(v.y.is_finite(), "non-finite y coordinate in {:?}", pline);
        assert!(v.bulge.is_finite(), "non-finite bulge in {:?}", pline);
    }
}

fn assert_shape_valid(shape: &Shape<f64>) {
    // Every boolean result should be immediately reusable as a Shape input, so verify the
    // orientation bins and spatial indexes instead of checking only area or membership.
    for ip in &shape.ccw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::CounterClockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    for ip in &shape.cw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::Clockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    let loop_count = shape.ccw_plines.len() + shape.cw_plines.len();
    if loop_count == 0 {
        assert!(
            shape.plines_index.bounds().is_none(),
            "empty shape should not have top-level bounds"
        );
        return;
    }

    let mut expected: Option<static_aabb2d_index::AABB<f64>> = None;
    for bounds in shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| ip.spatial_index.bounds().unwrap())
    {
        expected = Some(match expected {
            None => bounds,
            Some(curr) => static_aabb2d_index::AABB::new(
                curr.min_x.min(bounds.min_x),
                curr.min_y.min(bounds.min_y),
                curr.max_x.max(bounds.max_x),
                curr.max_y.max(bounds.max_y),
            ),
        });
    }

    let expected = expected.unwrap();
    let actual = shape.plines_index.bounds().unwrap();
    assert!(
        expected.min_x.fuzzy_eq_eps(actual.min_x, SHAPE_TEST_EPS)
            && expected.min_y.fuzzy_eq_eps(actual.min_y, SHAPE_TEST_EPS)
            && expected.max_x.fuzzy_eq_eps(actual.max_x, SHAPE_TEST_EPS)
            && expected.max_y.fuzzy_eq_eps(actual.max_y, SHAPE_TEST_EPS),
        "shape top-level index bounds do not match child loop bounds"
    );
}

fn assert_boolean_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    // Sampled membership is the semantic oracle: exact loop topology may vary, but each point
    // must obey the set operation computed from the original input shapes.
    for &(x, y) in samples {
        let in_a = shape_contains(a, x, y);
        let in_b = shape_contains(b, x, y);
        let expected = match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        };
        let actual = shape_contains(result, x, y);
        assert_eq!(
            actual,
            expected,
            "sample ({x}, {y}) mismatch for {op:?}: in_a={in_a}, in_b={in_b}, result={actual}, result_area={}",
            shape_signed_area(result)
        );
    }
}

fn assert_boolean_result(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    expected_ccw_count: usize,
    expected_cw_count: usize,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    assert_shape_valid(&result);
    assert_eq!(
        result.ccw_plines.len(),
        expected_ccw_count,
        "unexpected ccw loop count for {op:?}"
    );
    assert_eq!(
        result.cw_plines.len(),
        expected_cw_count,
        "unexpected cw loop count for {op:?}"
    );
    assert!(
        shape_signed_area(&result).fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {}",
        shape_signed_area(&result)
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

fn assert_boolean_area_and_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    assert_shape_valid(&result);
    assert!(
        shape_signed_area(&result).fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {}",
        shape_signed_area(&result)
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

fn assert_commutative_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    let ab = a.boolean(b, op);
    let ba = b.boolean(a, op);

    assert_shape_valid(&ab);
    assert_shape_valid(&ba);
    assert!(
        shape_signed_area(&ab).fuzzy_eq_eps(shape_signed_area(&ba), 1e-5),
        "commutative area mismatch for {op:?}: ab={}, ba={}",
        shape_signed_area(&ab),
        shape_signed_area(&ba)
    );
    assert_eq!(
        ab.ccw_plines.len(),
        ba.ccw_plines.len(),
        "commutative ccw loop count mismatch for {op:?}"
    );
    assert_eq!(
        ab.cw_plines.len(),
        ba.cw_plines.len(),
        "commutative cw loop count mismatch for {op:?}"
    );
    assert_boolean_samples(a, b, &ab, op, samples);
    assert_boolean_samples(b, a, &ba, op, samples);
}

fn assert_shapes_equivalent_by_samples(a: &Shape<f64>, b: &Shape<f64>, samples: &[(f64, f64)]) {
    assert_shape_valid(a);
    assert_shape_valid(b);
    assert!(
        shape_signed_area(a).fuzzy_eq_eps(shape_signed_area(b), 1e-5),
        "shape area mismatch: left={}, right={}",
        shape_signed_area(a),
        shape_signed_area(b)
    );
    assert_eq!(
        a.ccw_plines.len(),
        b.ccw_plines.len(),
        "ccw loop count mismatch"
    );
    assert_eq!(
        a.cw_plines.len(),
        b.cw_plines.len(),
        "cw loop count mismatch"
    );

    for &(x, y) in samples {
        assert_eq!(
            shape_contains(a, x, y),
            shape_contains(b, x, y),
            "sample ({x}, {y}) equivalence mismatch"
        );
    }
}

#[test]
fn shape_boolean_empty_identities_are_valid() {
    // Empty-shape identities catch missing unused-loop retention and accidental top-level bounds.
    let empty = Shape::<f64>::empty();
    let solid = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(-1.0, -1.0), (1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_shape_valid(&empty);
    assert_shape_valid(&solid);

    assert_boolean_result(&solid, &empty, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Xor, 100.0, 1, 0, &samples);
}

#[test]
fn shape_boolean_rectangle_containment_all_ops() {
    let outer = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let inner = Shape::from_plines(vec![create_rectangle(2.0, 2.0, 4.0, 4.0)]);
    let samples = [(1.0, 1.0), (3.0, 3.0), (8.0, 8.0), (12.0, 12.0)];

    assert_boolean_result(&outer, &inner, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::And, 4.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Not, 96.0, 1, 1, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Xor, 96.0, 1, 1, &samples);
}

#[test]
fn shape_boolean_edge_touching_rectangles_do_not_create_area() {
    let left = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let right = Shape::from_plines(vec![create_rectangle(10.0, 0.0, 20.0, 10.0)]);
    let samples = [(5.0, 5.0), (15.0, 5.0), (10.5, 5.0), (-1.0, 5.0)];

    assert_boolean_result(&left, &right, BooleanOp::Or, 200.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Xor, 200.0, 1, 0, &samples);
}

#[test]
fn shape_boolean_square_inside_hole_respects_empty_space() {
    // A positive island inside a hole is disjoint from the donut's filled area but must be
    // preserved by union and XOR as a nested positive loop.
    let donut = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_cw_rectangle(3.0, 3.0, 7.0, 7.0),
    ]);
    let island = Shape::from_plines(vec![create_rectangle(4.0, 4.0, 6.0, 6.0)]);
    let samples = [(1.0, 1.0), (3.5, 3.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(&donut, &island, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&island, &donut, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Not, 84.0, 1, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Xor, 88.0, 2, 1, &samples);
}

#[test]
fn shape_boolean_identical_rectangle_all_ops() {
    let rect = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_boolean_result(&rect, &rect, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::And, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_commutative_overlapping_rectangles() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let b = create_shape_rects(&[(5.0, 5.0, 15.0, 15.0)]);
    let samples = [(2.0, 2.0), (7.5, 7.5), (12.0, 12.0), (16.0, 16.0)];

    assert_commutative_samples(&a, &b, BooleanOp::Or, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::And, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::Xor, &samples);
}

#[test]
fn shape_boolean_multi_island_disjoint_identities() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let b = create_shape_rects(&[(100.0, 0.0, 110.0, 10.0)]);
    let samples = [(5.0, 5.0), (25.0, 5.0), (105.0, 5.0), (50.0, 5.0)];

    assert_boolean_result(&a, &b, BooleanOp::Or, 300.0, 3, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Not, 200.0, 2, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Xor, 300.0, 3, 0, &samples);
}

#[test]
fn shape_boolean_bridge_overlaps_two_islands() {
    // One input loop intersects two separate islands, forcing pairwise results to merge into a
    // single output component for union while preserving separated pieces for intersection.
    let islands = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let bridge = create_shape_rects(&[(5.0, -5.0, 25.0, 15.0)]);
    let samples = [
        (2.5, 5.0),
        (7.5, 5.0),
        (15.0, 0.0),
        (22.5, 5.0),
        (27.5, 5.0),
        (15.0, 12.5),
        (15.0, 20.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 500.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 100.0, 2, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 100.0, 2, 0, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::Or, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::And, &samples);
}

#[test]
fn shape_boolean_ring_difference_adds_second_hole() {
    // Difference against material inside the filled ring should add a new hole without disturbing
    // an existing unrelated hole.
    let ring = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let cutter_in_filled_ring = create_shape_rects(&[(1.0, 1.0, 2.0, 2.0)]);
    let samples = [(1.5, 1.5), (2.5, 2.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Not,
        83.0,
        1,
        2,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::And,
        1.0,
        1,
        0,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Xor,
        83.0,
        1,
        2,
        &samples,
    );
}

#[test]
fn shape_boolean_loop_order_and_start_index_are_irrelevant() {
    // Shape booleans should depend on geometry, not storage order or the chosen start vertex of a
    // closed loop. This protects the pairwise bookkeeping from index-order assumptions.
    let base_a = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_rectangle(20.0, 0.0, 30.0, 10.0),
    ]);
    let variant_a = Shape::from_plines(vec![
        rotate_pline_start(&create_rectangle(20.0, 0.0, 30.0, 10.0), 2),
        rotate_pline_start(&create_rectangle(0.0, 0.0, 10.0, 10.0), 3),
    ]);

    let base_b = Shape::from_plines(vec![create_rectangle(5.0, -5.0, 25.0, 15.0)]);
    let variant_b = Shape::from_plines(vec![rotate_pline_start(
        &create_rectangle(5.0, -5.0, 25.0, 15.0),
        1,
    )]);

    let samples = [
        (2.0, 2.0),
        (5.5, 5.5),
        (8.0, 8.0),
        (15.0, 1.0),
        (15.0, 4.0),
        (22.0, 5.0),
        (28.0, 5.0),
        (40.0, 40.0),
    ];

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let base_result = base_a.boolean(&base_b, op);
        let variant_result = variant_a.boolean(&variant_b, op);
        assert_shapes_equivalent_by_samples(&base_result, &variant_result, &samples);
        assert_boolean_samples(&base_a, &base_b, &base_result, op, &samples);
        assert_boolean_samples(&variant_a, &variant_b, &variant_result, op, &samples);
    }
}

#[test]
fn shape_boolean_chain_bridge_merges_three_islands() {
    // The bridge overlaps three islands transitively; a one-pass pairwise collector can easily
    // leave duplicate or partially merged loops here if final assembly is not robust.
    let islands = create_shape_rects(&[
        (0.0, 0.0, 6.0, 6.0),
        (10.0, 0.0, 16.0, 6.0),
        (20.0, 0.0, 26.0, 6.0),
    ]);
    let bridge = create_shape_rects(&[(3.0, -2.0, 23.0, 8.0)]);
    let samples = [
        (1.0, 1.0),
        (5.0, 1.0),
        (8.0, 0.0),
        (13.0, 3.0),
        (18.0, 0.0),
        (24.0, 3.0),
        (12.0, 7.0),
        (30.0, 3.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 236.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 72.0, 3, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 36.0, 2, 0, &samples);
    assert_boolean_area_and_samples(&islands, &bridge, BooleanOp::Xor, 164.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_splits_ring_without_filling_hole() {
    // Regression: a cutter that crosses a hole boundary must not cause the original hole to be
    // treated as filled material during intersection or difference assembly.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_result(&ring, &cutter, BooleanOp::And, 24.0, 1, 0, &samples);
    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Not, 104.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_xor() {
    // Regression for XOR's (A - B) union (B - A) composition: the B - A island lies inside the
    // merged hole from A - B, so union must keep both nested loops instead of filling the hole.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Xor, 112.0, &samples);
}

#[cfg(test)]
mod shape_boolean_proptests {
    use super::*;
    use proptest::prelude::*;

    fn rect_strategy() -> impl Strategy<Value = (f64, f64, f64, f64)> {
        // Integer coordinates keep expected areas exact while still exploring many placements.
        (-100i32..100, -100i32..100, 1i32..40, 1i32..40).prop_map(|(x, y, w, h)| {
            let xmin = f64::from(x);
            let ymin = f64::from(y);
            (xmin, ymin, xmin + f64::from(w), ymin + f64::from(h))
        })
    }

    fn rect_area(rect: (f64, f64, f64, f64)) -> f64 {
        (rect.2 - rect.0) * (rect.3 - rect.1)
    }

    fn rect_center(rect: (f64, f64, f64, f64)) -> (f64, f64) {
        ((rect.0 + rect.2) * 0.5, (rect.1 + rect.3) * 0.5)
    }

    fn outside_sample(rect: (f64, f64, f64, f64)) -> (f64, f64) {
        (rect.2 + 10.0, rect.3 + 10.0)
    }

    fn rect_overlap_area(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> f64 {
        let xmin = a.0.max(b.0);
        let ymin = a.1.max(b.1);
        let xmax = a.2.min(b.2);
        let ymax = a.3.min(b.3);
        if xmax <= xmin || ymax <= ymin {
            0.0
        } else {
            (xmax - xmin) * (ymax - ymin)
        }
    }

    fn overlapping_rect_pair_strategy()
    -> impl Strategy<Value = ((f64, f64, f64, f64), (f64, f64, f64, f64))> {
        // Bias toward guaranteed overlaps so area identities exercise clipping instead of only
        // unused-loop retention.
        (-50i32..50, -50i32..50, 6i32..40, 6i32..40, 1i32..6, 1i32..6).prop_map(
            |(x, y, w, h, inset_x, inset_y)| {
                let a = (
                    f64::from(x),
                    f64::from(y),
                    f64::from(x + w),
                    f64::from(y + h),
                );
                let b = (
                    f64::from(x + inset_x),
                    f64::from(y + inset_y),
                    f64::from(x + w + inset_x),
                    f64::from(y + h + inset_y),
                );
                (a, b)
            },
        )
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 64,
            failure_persistence: None,
            ..ProptestConfig::default()
        })]

        #[test]
        fn rectangle_empty_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let empty = Shape::<f64>::empty();
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &empty, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Not, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Xor, area, 1, 0, &samples);
        }

        #[test]
        fn translated_disjoint_rectangles_obey_basic_area_laws(
            rect in rect_strategy(),
            gap in 1i32..50,
            w2 in 1i32..40,
            h2 in 1i32..40,
        ) {
            let b = (
                rect.2 + f64::from(gap),
                rect.1,
                rect.2 + f64::from(gap + w2),
                rect.1 + f64::from(h2),
            );
            let a_shape = create_shape_rects(&[rect]);
            let b_shape = create_shape_rects(&[b]);
            let samples = [rect_center(rect), rect_center(b), outside_sample(b)];

            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Or,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Not, rect_area(rect), 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Xor,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
        }

        #[test]
        fn rectangle_self_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::And, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn overlapping_rectangles_obey_area_identities((a, b) in overlapping_rect_pair_strategy()) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let overlap = rect_overlap_area(a, b);
            let union_area = rect_area(a) + rect_area(b) - overlap;
            let xor_area = rect_area(a) + rect_area(b) - 2.0 * overlap;
            let samples = [
                ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5),
                ((a.2 + b.2) * 0.5, (a.3 + b.3) * 0.5),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
                outside_sample(b),
            ];

            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Or, union_area, 1, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, overlap, 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Not,
                rect_area(a) - overlap,
                1,
                0,
                &samples,
            );
            assert_boolean_area_and_samples(&a_shape, &b_shape, BooleanOp::Xor, xor_area, &samples);
        }
    }
}

#[test]
fn shape_empty_union_returns_self() {
    let shape_empty = Shape::<f64>::empty();
    let shape_nonempty = {
        let mut p = Polyline::new_closed();
        p.add(0.0, 0.0, 0.0);
        p.add(10.0, 0.0, 0.0);
        p.add(10.0, 10.0, 0.0);
        p.add(0.0, 10.0, 0.0);
        Shape::from_plines(vec![p])
    };
    let union_result = shape_empty.union(&shape_nonempty);
    // Expect same shape as `shape_nonempty`:
    assert_eq!(
        union_result.ccw_plines.len(),
        shape_nonempty.ccw_plines.len()
    );
    assert_eq!(union_result.cw_plines.len(), shape_nonempty.cw_plines.len());
}

#[test]
fn shape_disjoint_union_keeps_both() {
    // Two squares far apart, union => 2 loops in resulting shape
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let union_result = shape1.union(&shape2);
    // Should have 2 separate loops, both ccw:
    assert_eq!(union_result.ccw_plines.len(), 2);
    assert!(union_result.cw_plines.is_empty());
}

#[test]
fn shape_contains_shape_union_gives_first() {
    // Big square fully containing a smaller square => union is just bigger square
    let big_sq = pline_closed![
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (100.0, 100.0, 0.0),
        (0.0, 100.0, 0.0)
    ];
    let small_sq = pline_closed![
        (10.0, 10.0, 0.0),
        (20.0, 10.0, 0.0),
        (20.0, 20.0, 0.0),
        (10.0, 20.0, 0.0)
    ];
    let shape_big = Shape::from_plines(vec![big_sq.clone()]);
    let shape_small = Shape::from_plines(vec![small_sq]);

    let union_result = shape_big.union(&shape_small);
    // Should have same bounding box and area as `big_sq`
    assert_eq!(union_result.ccw_plines.len(), 1);
    let union_pline = &union_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(union_pline.area(), big_sq.area());
}

#[test]
fn shape_difference_completely_disjoint_no_intersection() {
    // Two squares far apart => difference => same as first
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1.clone()]);
    let shape2 = Shape::from_plines(vec![square2]);
    let diff_result = shape1.difference(&shape2);
    assert_eq!(diff_result.ccw_plines.len(), 1);
    assert!(diff_result.cw_plines.is_empty());

    let out_pline = &diff_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(out_pline.area(), square1.area());
}

#[test]
fn shape_intersection_disjoint_is_empty() {
    // Two squares far apart => intersection => empty
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let intersect_result = shape1.intersection(&shape2);
    // Should be empty
    assert!(intersect_result.ccw_plines.is_empty());
    assert!(intersect_result.cw_plines.is_empty());
}

#[test]
fn shape_open_pline_included_in_boolean() {
    // If shape has open polyline, those won't be closed => won't form loops
    // but let's just confirm no panic:
    let open_pl = pline_open![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![open_pl]);
    let shape_empty = Shape::<f64>::empty();
    let union_result = shape.union(&shape_empty);
    // union => same as shape
    // though the shape is "not fully closed," area = 0, etc.
    assert_eq!(union_result.ccw_plines.len(), shape.ccw_plines.len());
}

#[test]
fn shape_parallel_offset_outward() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(-1.0, Default::default());
    // Expect single loop with bigger bounding box
    assert_eq!(offset_result.ccw_plines.len(), 1);
    let out_pline = &offset_result.ccw_plines[0].polyline;
    assert!(out_pline.extents().unwrap().max_x > 4.9);
    assert!(out_pline.extents().unwrap().max_y > 2.9);
}

#[test]
fn shape_parallel_offset_inward_collapses() {
    // Inward offset that collapses rectangle
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(2.0, Default::default());
    // Collapsed => empty
    assert!(offset_result.ccw_plines.is_empty());
    assert!(offset_result.cw_plines.is_empty());
}

#[test]
fn shape_translate_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.translate_mut(5.0, 5.0);
    // bounding box now from (5,5) to (15,15)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 5.0);
    assert_fuzzy_eq!(aabb.min_y, 5.0);
    assert_fuzzy_eq!(aabb.max_x, 15.0);
    assert_fuzzy_eq!(aabb.max_y, 15.0);
}

#[test]
fn shape_scale_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.scale_mut(2.0);
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, 0.0);
    assert_fuzzy_eq!(aabb.max_x, 20.0);
    assert_fuzzy_eq!(aabb.max_y, 20.0);
}

#[test]
fn shape_mirror_x_works() {
    // Mirror about X (meaning y -> -y)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.mirror_x_mut();
    // bounding box now from (0, -2) to (2, 0)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, -2.0);
    assert_fuzzy_eq!(aabb.max_x, 2.0);
    assert_fuzzy_eq!(aabb.max_y, 0.0);
}

#[test]
fn shape_center_works() {
    // shape is from (0,0) to (10,10)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.center_mut();
    let aabb = shape.plines_index.bounds().unwrap();
    // bounding box is now from (-5,-5) to (5,5)
    assert_fuzzy_eq!(aabb.min_x, -5.0);
    assert_fuzzy_eq!(aabb.min_y, -5.0);
    assert_fuzzy_eq!(aabb.max_x, 5.0);
    assert_fuzzy_eq!(aabb.max_y, 5.0);
}

#[test]
fn shape_transform_mut() {
    // We'll apply a 2D transform [a b; c d] + translation
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    // transform is M = [2  0; 0 2], plus (10,10) translation
    // so a=2,b=0,c=0,d=2,tx=10,ty=10
    shape.transform_mut(2.0, 0.0, 0.0, 2.0, 10.0, 10.0);

    // bounding box now from (10,10) to (12,12)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 10.0);
    assert_fuzzy_eq!(aabb.min_y, 10.0);
    assert_fuzzy_eq!(aabb.max_x, 12.0);
    assert_fuzzy_eq!(aabb.max_y, 12.0);
}

#[test]
fn transform_builder_scale_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // scale by factor of 2 about origin => extents from (0,0) to (20,10)
    let mut shape_copy = shape.clone();
    shape_copy.scale_mut(2.0);
    assert_extents_fuzzy_eq(&shape_copy, 0.0, 0.0, 20.0, 10.0, 1e-5);
}

#[test]
fn transform_builder_translate_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // translate by (15, -2)
    let mut shape_copy = shape.clone();
    shape_copy.translate_mut(15.0, -2.0);
    // new bounding box => from (15,-2) to (25,3)
    assert_extents_fuzzy_eq(&shape_copy, 15.0, -2.0, 25.0, 3.0, 1e-5);
}

#[test]
fn transform_builder_center_test() {
    // Make a rectangle from (0,0) to (10,10) => extents from (0,0) to (10,10)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 10.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // center => bounding box is from (-5,-5) to (5,5)
    let mut shape_copy = shape.clone();
    shape_copy.center_mut();
    assert_extents_fuzzy_eq(&shape_copy, -5.0, -5.0, 5.0, 5.0, 1e-5);
}

#[test]
fn transform_builder_mirror_x_test() {
    // rectangle from (1.0,1.0) to (4.0,3.0)
    let rect_pline = create_rectangle(1.0, 1.0, 4.0, 3.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // mirror_x => flip in place (y -> -y), so bounding box => min_y is -3.0, max_y is -1.0
    let mut shape_copy = shape.clone();
    shape_copy.mirror_x_mut();
    // bounding box => x from (1,4) to (1,4) unchanged, y from ( -3, -1)
    assert_extents_fuzzy_eq(&shape_copy, 1.0, -3.0, 4.0, -1.0, 1e-5);
}

#[test]
fn transform_builder_rotate_test() {
    // rectangle from (0.0,0.0) to (10.0, 0.0) => extents from (0,0) to (10,0) plus the small y range
    let mut rect_pline = Polyline::new_closed();
    rect_pline.add(0.0, 0.0, 0.0);
    rect_pline.add(10.0, 0.0, 0.0);
    rect_pline.add(10.0, 2.0, 0.0);
    rect_pline.add(0.0, 2.0, 0.0);

    let shape = Shape::from_plines(vec![rect_pline]);
    // rotate 90 degrees about (0,0)
    let mut shape_copy = shape.clone();
    shape_copy.rotate_mut(PI * 0.5);

    // after 90 deg rotation:
    //  originally x from [0..10], y from [0..2]
    //  becomes x from [-2..0], y from [0..10]
    assert_extents_fuzzy_eq(&shape_copy, -2.0, 0.0, 0.0, 10.0, 1e-5);
}

#[test]
fn shape_offset_simple_test() {
    // Make a single circle-like shape from approx. circle
    let circle_pline = create_approx_circle(0.0, 0.0, 5.0);
    let shape = Shape::from_plines(vec![circle_pline]);

    // offset outward by 2 => bounding box with radius ~ 7
    // bounding box => from (-7, -0.0) to (7, 0.0) along major axis for our 2-vertex half circle
    // but user’s shape might be incomplete, so we just check that min_x ~ -7.0 or so.
    let offset_opts = ShapeOffsetOptions::default();
    let result = shape.parallel_offset(-2.0, offset_opts);
    assert_eq!(result.ccw_plines.len(), 1);
    let final_bb = result.plines_index.bounds().unwrap();
    // radius => 7 => min_x => -7, max_x => 7
    // we have half circle, so final bounding box might show up. We'll do a small check:
    // not exact because of half circle, but let's do an approximate check
    assert!(
        final_bb.min_x < -6.95 && final_bb.min_x > -7.05,
        "expected min_x ~ -7.0, got {}",
        final_bb.min_x
    );
}
