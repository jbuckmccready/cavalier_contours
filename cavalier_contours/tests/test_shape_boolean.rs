use cavalier_contours::core::traits::FuzzyEq;
use cavalier_contours::polyline::{PlineSource, PlineSourceMut, Polyline};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};
use cavalier_contours::{assert_fuzzy_eq, pline_closed, pline_open};
use std::f64::consts::PI;

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
    // just a 2-vertex half-circle for demonstration
    // can be replaced with more segments for better approximation
    let mut pl = Polyline::new_closed();
    // half circle going from left to right
    // bulge for half circle = tan(π/4) = 1.0
    pl.add(center_x - radius, center_y, 1.0);
    pl.add(center_x + radius, center_y, 1.0);
    pl
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
