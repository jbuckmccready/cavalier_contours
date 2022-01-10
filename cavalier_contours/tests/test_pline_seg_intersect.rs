use cavalier_contours::{
    core::math::{bulge_from_angle, Vector2},
    polyline::{pline_seg_intr, PlineSegIntr::*, PlineVertex},
};
use std::f64::consts::FRAC_PI_2;

macro_rules! assert_case_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (NoIntersect, NoIntersect) => {}
            (TangentIntersect { point: a1 }, TangentIntersect { point: a2 })
            | (OneIntersect { point: a1 }, OneIntersect { point: a2 })
                if a1.fuzzy_eq(a2) => {}
            (
                TwoIntersects {
                    point1: a1,
                    point2: b1,
                },
                TwoIntersects {
                    point1: a2,
                    point2: b2,
                },
            )
            | (
                OverlappingLines {
                    point1: a1,
                    point2: b1,
                },
                OverlappingLines {
                    point1: a2,
                    point2: b2,
                },
            )
            | (
                OverlappingArcs {
                    point1: a1,
                    point2: b1,
                },
                OverlappingArcs {
                    point1: a2,
                    point2: b2,
                },
            ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
            _ => panic!(
                "intersect cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        };
    };
}

#[test]
fn arc_line_no_intersect() {
    let v1 = PlineVertex::new(0.0, 0.0, 1.0);
    let v2 = PlineVertex::new(2.0, 0.0, 0.0);
    let u1 = PlineVertex::new(0.0, 1.0, 0.0);
    let u2 = PlineVertex::new(2.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn line_arc_no_intersect() {
    let v1 = PlineVertex::new(0.0, 1.0, 0.0);
    let v2 = PlineVertex::new(2.0, 3.0, 0.0);
    let u1 = PlineVertex::new(0.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(result, NoIntersect::<f64>);
}

#[test]
fn overlapping_lines() {
    let v1 = PlineVertex::new(3.0, 3.0, 0.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 0.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingLines {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(2.0, 2.0)
        }
    );
}

#[test]
fn overlapping_lines_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 0.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(2.0, 2.0, 0.0);
    let u2 = PlineVertex::new(1.0, 1.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingLines {
            point1: Vector2::new(2.0, 2.0),
            point2: Vector2::new(1.0, 1.0)
        }
    );
}

#[test]
fn overlapping_same_arcs() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn overlapping_same_arcs_reverse_dir() {
    let v1 = PlineVertex::new(3.0, 3.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn arc_arc_end_points_touch() {
    let v1 = PlineVertex::new(3.0, 3.0, 1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn arc_arc_end_points_touch_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, -1.0);
    let v2 = PlineVertex::new(3.0, 3.0, 0.0);
    let u1 = PlineVertex::new(1.0, 1.0, 1.0);
    let u2 = PlineVertex::new(3.0, 3.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        TwoIntersects {
            point1: Vector2::new(1.0, 1.0),
            point2: Vector2::new(3.0, 3.0)
        }
    );
}

#[test]
fn arc2_within_arc1_overlapping() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(2.0, 0.0, bulge);
    let u2 = PlineVertex::new(3.0, 1.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc1_within_arc2_overlapping() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(2.0, 0.0, bulge);
    let u2 = PlineVertex::new(3.0, 1.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc2_within_arc1_overlapping_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(3.0, 1.0, -bulge);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}

#[test]
fn arc1_within_arc2_overlapping_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let bulge = bulge_from_angle(FRAC_PI_2);
    let u1 = PlineVertex::new(3.0, 1.0, -bulge);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_flipped() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc2_reverse_dir() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 2.0, -1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc2_reverse_dir_flipped() {
    let v1 = PlineVertex::new(1.0, 1.0, 1.0);
    let v2 = PlineVertex::new(3.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 2.0, -1.0);
    let u2 = PlineVertex::new(2.0, 0.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc1_reverse_dir() {
    let v1 = PlineVertex::new(3.0, 1.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(v1, v2, u1, u2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(2.0, 0.0),
            point2: Vector2::new(3.0, 1.0)
        }
    );
}

#[test]
fn arc_arc_partial_overlap_arc1_reverse_dir_flipped() {
    let v1 = PlineVertex::new(3.0, 1.0, -1.0);
    let v2 = PlineVertex::new(1.0, 1.0, 0.0);

    let u1 = PlineVertex::new(2.0, 0.0, 1.0);
    let u2 = PlineVertex::new(2.0, 2.0, 0.0);
    let result = pline_seg_intr(u1, u2, v1, v2, 1e-5);
    assert_case_eq!(
        result,
        OverlappingArcs {
            point1: Vector2::new(3.0, 1.0),
            point2: Vector2::new(2.0, 0.0)
        }
    );
}
