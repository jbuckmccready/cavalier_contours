use cavalier_contours::{
    core::traits::FuzzyEq,
    polyline::{
        BooleanOp, BooleanResultInfo, PlineBooleanOptions, PlineSource, PlineSourceMut, Polyline,
    },
};

fn main() {
    union_operations();
    intersection_operations();
    difference_operations();
    special_cases();
}

fn union_operations() {
    println!("Testing union (OR) operations...");

    // Overlapping rectangles
    let mut rect1 = Polyline::new();
    rect1.add(0.0, 0.0, 0.0);
    rect1.add(10.0, 0.0, 0.0);
    rect1.add(10.0, 10.0, 0.0);
    rect1.add(0.0, 10.0, 0.0);
    rect1.set_is_closed(true);

    let mut rect2 = Polyline::new();
    rect2.add(5.0, 5.0, 0.0);
    rect2.add(15.0, 5.0, 0.0);
    rect2.add(15.0, 15.0, 0.0);
    rect2.add(5.0, 15.0, 0.0);
    rect2.set_is_closed(true);

    let union_result = rect1.boolean(&rect2, BooleanOp::Or);
    assert!(
        matches!(
            union_result.result_info,
            BooleanResultInfo::Intersected
                | BooleanResultInfo::Pline1InsidePline2
                | BooleanResultInfo::Pline2InsidePline1
        ),
        "Union result should indicate intersection or containment"
    );
    assert!(
        !union_result.pos_plines.is_empty(),
        "Union should produce at least one positive polyline"
    );
    assert!(
        union_result.neg_plines.is_empty(),
        "Union should not produce negative (hole) polylines"
    );

    // Union area should be less than sum (due to overlap)
    let total_union_area: f64 = union_result
        .pos_plines
        .iter()
        .map(|p| p.pline.area())
        .sum::<f64>();

    assert!(
        total_union_area > 100.0,
        "Union area should be greater than single square (100)"
    );
    assert!(
        total_union_area < 200.0,
        "Union area should be less than sum of both squares (200)"
    );
    println!(
        "Union of overlapping rectangles: area = {:.2}, result info: {:?}",
        total_union_area, union_result.result_info
    );

    println!("Union operations completed successfully!\n");
}

fn intersection_operations() {
    println!("Testing intersection (AND) operations...");

    // Overlapping rectangles intersection
    let mut rect1 = Polyline::new();
    rect1.add(0.0, 0.0, 0.0);
    rect1.add(10.0, 0.0, 0.0);
    rect1.add(10.0, 10.0, 0.0);
    rect1.add(0.0, 10.0, 0.0);
    rect1.set_is_closed(true);

    let mut rect2 = Polyline::new();
    rect2.add(5.0, 5.0, 0.0);
    rect2.add(15.0, 5.0, 0.0);
    rect2.add(15.0, 15.0, 0.0);
    rect2.add(5.0, 15.0, 0.0);
    rect2.set_is_closed(true);

    let intersection_result = rect1.boolean(&rect2, BooleanOp::And);
    assert!(
        matches!(
            intersection_result.result_info,
            BooleanResultInfo::Intersected
                | BooleanResultInfo::Pline1InsidePline2
                | BooleanResultInfo::Pline2InsidePline1
        ),
        "Intersection result should indicate intersection or containment"
    );

    let intersection_area: f64 = intersection_result
        .pos_plines
        .iter()
        .map(|p| p.pline.area())
        .sum();
    assert!(
        intersection_area > 0.0,
        "Intersection area should be positive (overlapping rectangles)"
    );
    assert!(
        intersection_area < 100.0,
        "Intersection area should be less than full square (100)"
    );
    println!("Intersection of overlapping rectangles: area = {intersection_area:.2}");

    // Non-overlapping rectangles (no intersection)
    let mut rect3 = Polyline::new();
    rect3.add(20.0, 0.0, 0.0);
    rect3.add(30.0, 0.0, 0.0);
    rect3.add(30.0, 10.0, 0.0);
    rect3.add(20.0, 10.0, 0.0);
    rect3.set_is_closed(true);

    let no_intersection = rect1.boolean(&rect3, BooleanOp::And);
    assert_eq!(
        no_intersection.pos_plines.len(),
        0,
        "Non-overlapping rectangles should have no intersection polylines"
    );
    println!("Non-overlapping rectangles: no intersection as expected");

    println!("Intersection operations completed successfully!\n");
}

fn difference_operations() {
    println!("Testing difference (NOT) operations...");

    // Create outer and inner rectangles for subtraction
    let mut outer_rect = Polyline::new();
    outer_rect.add(0.0, 0.0, 0.0);
    outer_rect.add(20.0, 0.0, 0.0);
    outer_rect.add(20.0, 20.0, 0.0);
    outer_rect.add(0.0, 20.0, 0.0);
    outer_rect.set_is_closed(true);

    let mut inner_rect = Polyline::new();
    inner_rect.add(5.0, 5.0, 0.0);
    inner_rect.add(15.0, 5.0, 0.0);
    inner_rect.add(15.0, 15.0, 0.0);
    inner_rect.add(5.0, 15.0, 0.0);
    inner_rect.set_is_closed(true);

    let difference_result = outer_rect.boolean(&inner_rect, BooleanOp::Not);
    assert!(
        matches!(
            difference_result.result_info,
            BooleanResultInfo::Pline2InsidePline1
        ),
        "Difference result should indicate second polyline is inside first"
    );

    // expect 1 area and 1 hole
    assert_eq!(
        difference_result.pos_plines.len(),
        1,
        "Difference should produce exactly 1 positive (area) polyline"
    );
    assert_eq!(
        difference_result.neg_plines.len(),
        1,
        "Difference should produce exactly 1 negative (hole) polyline"
    );

    // `pos_plines` are areas, `neg_plines` are holes, so we subtract holes to get area
    let remaining_area: f64 = difference_result
        .pos_plines
        .iter()
        .map(|p| p.pline.area())
        .sum::<f64>()
        - difference_result
            .neg_plines
            .iter()
            .map(|p| p.pline.area())
            .sum::<f64>();

    assert!(
        remaining_area.fuzzy_eq(outer_rect.area() - inner_rect.area()),
        "Remaining area should equal outer minus inner area"
    );
    println!("Outer - inner rectangle: remaining area = {remaining_area:.2}");

    // Reverse difference (smaller - larger) should be empty or minimal
    let reverse_diff = inner_rect.boolean(&outer_rect, BooleanOp::Not);
    assert_eq!(
        reverse_diff.pos_plines.len(),
        0,
        "Reverse difference should have no positive polylines"
    );
    assert_eq!(
        reverse_diff.neg_plines.len(),
        0,
        "Reverse difference should have no negative polylines"
    );
    println!("Inner - outer rectangle: empty result as expected");

    println!("Difference operations completed successfully!\n");
}

// XOR test combined with special cases for brevity

fn special_cases() {
    println!("Testing special cases and XOR operations...");

    // Identical rectangles (complete overlap)
    let mut rect1 = Polyline::new();
    rect1.add(0.0, 0.0, 0.0);
    rect1.add(10.0, 0.0, 0.0);
    rect1.add(10.0, 10.0, 0.0);
    rect1.add(0.0, 10.0, 0.0);
    rect1.set_is_closed(true);

    let rect2 = rect1.clone();

    let identical_union = rect1.boolean(&rect2, BooleanOp::Or);
    assert!(
        matches!(identical_union.result_info, BooleanResultInfo::Overlapping),
        "Union of identical rectangles should have Overlapping result"
    );
    assert!(
        !identical_union.pos_plines.is_empty(),
        "Union of identical rectangles should produce positive polylines"
    );
    println!("Union of identical rectangles: overlapping result");

    // XOR of identical shapes should be empty
    let identical_xor = rect1.boolean(&rect2, BooleanOp::Xor);
    assert!(
        identical_xor.pos_plines.is_empty(),
        "XOR of identical rectangles should produce no polylines"
    );
    println!("XOR of identical rectangles: empty result as expected");

    // Custom options test
    let mut options = PlineBooleanOptions::new();
    options.pos_equal_eps = f64::fuzzy_epsilon() * 10.0;

    let custom_result = rect1.boolean_opt(&rect2, BooleanOp::And, &options);
    assert!(
        matches!(custom_result.result_info, BooleanResultInfo::Overlapping),
        "Custom options should still produce Overlapping result for identical rectangles"
    );
    println!("Custom options test: overlapping result with modified epsilon");

    println!(
        "NOTE: boolean operations do not work with open polylines, so no open polyline tests here."
    );

    println!("Special cases and XOR operations completed successfully!\n");
}
