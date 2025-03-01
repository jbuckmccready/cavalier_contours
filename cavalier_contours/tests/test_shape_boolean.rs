use cavalier_contours::polyline::{
    PlineSource, PlineSourceMut, Polyline,
};
use cavalier_contours::shape_algorithms::{
    Shape,
};
use cavalier_contours::{assert_fuzzy_eq, pline_closed, pline_open};

#[test]
fn shape_empty_union_returns_none() {
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
    // Expect same shape as `shape_empty`:
    assert_eq!(union_result.ccw_plines.len(), shape_empty.ccw_plines.len());
    assert_eq!(union_result.cw_plines.len(), shape_empty.cw_plines.len());
}

#[test]
fn shape_disjoint_union_keeps_both() {
    // Two squares far apart, union => 2 loops in resulting shape
    let square1 = pline_closed![
        (0.0,  0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0,10.0,0.0),
        (0.0, 10.0,0.0)
    ];
    let square2 = pline_closed![
        (100.0,100.0,0.0),
        (110.0,100.0,0.0),
        (110.0,110.0,0.0),
        (100.0,110.0,0.0)
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
        (0.0,  0.0,  0.0),
        (100.0,0.0,  0.0),
        (100.0,100.0,0.0),
        (0.0,  100.0,0.0)
    ];
    let small_sq = pline_closed![
        (10.0,10.0,0.0),
        (20.0,10.0,0.0),
        (20.0,20.0,0.0),
        (10.0,20.0,0.0)
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
        (0.0,  0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0,10.0,0.0),
        (0.0, 10.0,0.0)
    ];
    let square2 = pline_closed![
        (100.0,100.0,0.0),
        (110.0,100.0,0.0),
        (110.0,110.0,0.0),
        (100.0,110.0,0.0)
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
        (0.0,  0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0,10.0,0.0),
        (0.0, 10.0,0.0)
    ];
    let square2 = pline_closed![
        (100.0,100.0,0.0),
        (110.0,100.0,0.0),
        (110.0,110.0,0.0),
        (100.0,110.0,0.0)
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
        (0.0,0.0,0.0),
        (10.0,0.0,0.0),
        (10.0,10.0,0.0),
        (0.0,10.0,0.0)
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
        (0.0,0.0,0.0),
        (4.0,0.0,0.0),
        (4.0,2.0,0.0),
        (0.0,2.0,0.0)
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
        (0.0,0.0,0.0),
        (4.0,0.0,0.0),
        (4.0,2.0,0.0),
        (0.0,2.0,0.0)
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
        (0.0,0.0,0.0),
        (10.0,0.0,0.0),
        (10.0,10.0,0.0),
        (0.0,10.0,0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.translate_mut(5.0, 5.0);
    // bounding box now from (5,5) to (15,15)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x,5.0);
    assert_fuzzy_eq!(aabb.min_y,5.0);
    assert_fuzzy_eq!(aabb.max_x,15.0);
    assert_fuzzy_eq!(aabb.max_y,15.0);
}

#[test]
fn shape_scale_works() {
    let rect = pline_closed![
        (0.0,0.0,0.0),
        (10.0,0.0,0.0),
        (10.0,10.0,0.0),
        (0.0,10.0,0.0)
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
        (0.0,0.0,0.0),
        (2.0,0.0,0.0),
        (2.0,2.0,0.0),
        (0.0,2.0,0.0)
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
        (0.0,0.0,0.0),
        (10.0,0.0,0.0),
        (10.0,10.0,0.0),
        (0.0,10.0,0.0)
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
        (0.0,0.0,0.0),
        (1.0,0.0,0.0),
        (1.0,1.0,0.0),
        (0.0,1.0,0.0)
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
