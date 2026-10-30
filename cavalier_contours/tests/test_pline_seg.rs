#![allow(
    clippy::float_cmp,
    reason = "exact float comparisons are intentional in these tests"
)]

use cavalier_contours::{
    assert_fuzzy_eq,
    core::{math::Vector2, traits::FuzzyEq},
    polyline::{
        PlineVertex, SplitResult, dist_from_segment_start, seg_arc_radius_and_center,
        seg_bounding_box, seg_closest_point, seg_distance_is_greater_than,
        seg_fast_approx_bounding_box, seg_length, seg_midpoint, seg_split_at_point,
        seg_tangent_vector,
    },
};
use static_aabb2d_index::AABB;
use std::f64::consts::{FRAC_PI_2, PI, SQRT_2};

const POS_EPS: f64 = 1e-12;

fn arc_from_unit_circle(bulge: f64) -> (PlineVertex<f64>, PlineVertex<f64>) {
    let sweep = 4.0 * bulge.atan();
    (
        PlineVertex::new(1.0, 0.0, bulge),
        PlineVertex::new(sweep.cos(), sweep.sin(), 0.0),
    )
}

fn point_on_unit_arc(bulge: f64, fraction: f64) -> Vector2<f64> {
    let sweep = 4.0 * bulge.atan();
    let angle = sweep * fraction;
    Vector2::new(angle.cos(), angle.sin())
}

#[test]
fn seg_arc_radius_and_center_half_circle_on_x_axis() {
    // Chord from (0, 0) to (4, 0) with bulge 1 is a semicircle: radius = chord / (2|bulge|) = 2.
    let v1 = PlineVertex::new(0.0, 0.0, 1.0);
    let v2 = PlineVertex::new(4.0, 0.0, 0.0);
    let (radius, center) = seg_arc_radius_and_center(v1, v2);
    assert_fuzzy_eq!(radius, 2.0);
    assert_fuzzy_eq!(center, Vector2::new(2.0, 0.0));
}

#[test]
fn seg_closest_point_on_line_uses_perpendicular_foot_or_endpoints() {
    let v1 = PlineVertex::new(0.0, 0.0, 0.0);
    let v2 = PlineVertex::new(4.0, 0.0, 0.0);

    // Orthogonal projection of (2, 3) onto the x-axis segment is (2, 0).
    assert_fuzzy_eq!(
        seg_closest_point(v1, v2, Vector2::new(2.0, 3.0), POS_EPS),
        Vector2::new(2.0, 0.0)
    );

    // Past the start, closest point is v1.
    assert_fuzzy_eq!(
        seg_closest_point(v1, v2, Vector2::new(-2.0, 1.0), POS_EPS),
        v1.pos()
    );

    // Past the end, closest point is v2.
    assert_fuzzy_eq!(
        seg_closest_point(v1, v2, Vector2::new(6.0, -1.0), POS_EPS),
        v2.pos()
    );
}

#[test]
fn seg_tangent_vector_on_line_matches_chord_direction() {
    let v1 = PlineVertex::new(1.0, 2.0, 0.0);
    let v2 = PlineVertex::new(5.0, 6.0, 0.0);
    let chord = v2.pos() - v1.pos();
    for t in [0.0, 0.25, 0.5, 1.0] {
        let point = v1.pos() + chord.scale(t);
        assert_fuzzy_eq!(seg_tangent_vector(v1, v2, point), chord);
    }
}

#[test]
fn seg_distance_is_greater_than_on_line_matches_perpendicular_distance() {
    let v1 = PlineVertex::new(0.0, 0.0, 0.0);
    let v2 = PlineVertex::new(4.0, 0.0, 0.0);
    let point = Vector2::new(2.0, 2.0);
    // Squared perpendicular distance is cross²/chord² = 2²; comparison uses strict `>`.
    assert!(!seg_distance_is_greater_than(v1, v2, point, 2.0, POS_EPS));
    assert!(seg_distance_is_greater_than(
        v1,
        v2,
        point,
        2.0 - 1e-9,
        POS_EPS
    ));
    assert!(!seg_distance_is_greater_than(
        v1,
        v2,
        point,
        2.0 + 1e-9,
        POS_EPS
    ));
}

#[test]
fn line_segment_bounding_boxes_match_endpoints() {
    let v1 = PlineVertex::new(-1.0, 3.0, 0.0);
    let v2 = PlineVertex::new(2.0, -4.0, 0.0);
    let expected = AABB::new(-1.0, -4.0, 2.0, 3.0);
    assert_eq!(seg_bounding_box(v1, v2), expected);
    assert_eq!(seg_fast_approx_bounding_box(v1, v2), expected);
}

#[test]
fn seg_length_line_and_coincident_vertices() {
    let v1 = PlineVertex::new(2.0, 2.0, 0.0);
    let v2 = PlineVertex::new(4.0, 4.0, 0.0);
    // Euclidean distance along the diagonal is 2 * sqrt(2).
    assert_fuzzy_eq!(seg_length(v1, v2), 2.0 * SQRT_2);

    let duplicate = PlineVertex::new(1.0, -2.0, 0.5);
    assert_fuzzy_eq!(seg_length(duplicate, duplicate), 0.0);
}

#[test]
fn dist_from_segment_start_on_line_is_projection_length() {
    let v1 = PlineVertex::new(0.0, 0.0, 0.0);
    let v2 = PlineVertex::new(3.0, 4.0, 0.0);
    // Unit direction (0.6, 0.8); point two units from v1 along the segment.
    let point = Vector2::new(1.2, 1.6);
    assert_fuzzy_eq!(dist_from_segment_start(v1, v2, point, POS_EPS), 2.0);
    assert_fuzzy_eq!(
        dist_from_segment_start(v1, v2, v2.pos(), POS_EPS),
        seg_length(v1, v2)
    );
}

#[test]
fn seg_midpoint_on_line_is_arithmetic_mean() {
    let v1 = PlineVertex::new(-3.0, 1.0, 0.0);
    let v2 = PlineVertex::new(5.0, -7.0, 0.0);
    assert_fuzzy_eq!(seg_midpoint(v1, v2), Vector2::new(1.0, -3.0));
}

#[test]
fn seg_split_at_point_on_line_preserves_start_and_sets_zero_bulge_at_split() {
    let v1 = PlineVertex::new(0.0, 0.0, 0.0);
    let v2 = PlineVertex::new(8.0, 0.0, 0.0);
    let point = Vector2::new(3.0, 0.0);
    let SplitResult {
        updated_start,
        split_vertex,
    } = seg_split_at_point(v1, v2, point, POS_EPS);
    assert_eq!(updated_start, v1);
    assert_fuzzy_eq!(split_vertex, PlineVertex::new(3.0, 0.0, 0.0));
}

#[test]
fn seg_split_at_point_on_arc_at_endpoints_matches_documented_degenerate_cases() {
    let (v1, v2) = arc_from_unit_circle(1.0);

    let at_start = seg_split_at_point(v1, v2, v1.pos(), POS_EPS);
    assert_fuzzy_eq!(at_start.updated_start.pos(), v1.pos());
    assert_fuzzy_eq!(at_start.updated_start.bulge, 0.0);
    assert_fuzzy_eq!(at_start.split_vertex.pos(), v1.pos());
    assert_fuzzy_eq!(at_start.split_vertex.bulge, v1.bulge);

    let at_end = seg_split_at_point(v1, v2, v2.pos(), POS_EPS);
    assert_eq!(at_end.updated_start, v1);
    assert_fuzzy_eq!(at_end.split_vertex, PlineVertex::new(v2.x, v2.y, 0.0));
}

#[test]
fn seg_split_at_point_subarc_lengths_sum_to_original_length() {
    for bulge in [-1.0, -0.5, -1e-7, 1e-7, 0.5, 1.0] {
        let (v1, v2) = arc_from_unit_circle(bulge);
        let total = seg_length(v1, v2);
        for fraction in [0.1, 0.35, 0.5, 0.75, 0.9] {
            let point = point_on_unit_arc(bulge, fraction);
            let split = seg_split_at_point(v1, v2, point, POS_EPS);
            let first = seg_length(split.updated_start, split.split_vertex);
            let second = seg_length(split.split_vertex, v2);
            let sum = first + second;
            assert!(
                sum.fuzzy_eq_eps(total, 2e-13 * total.max(1.0)),
                "{bulge} at {fraction}: {sum} != {total}"
            );
        }
    }
}

#[test]
fn dist_from_segment_start_at_arc_midpoint_is_half_length() {
    for bulge in [-FRAC_PI_2.tan(), -1.0, 1.0, FRAC_PI_2.tan()] {
        let (v1, v2) = arc_from_unit_circle(bulge);
        let midpoint = point_on_unit_arc(bulge, 0.5);
        let total = seg_length(v1, v2);
        assert_fuzzy_eq!(
            dist_from_segment_start(v1, v2, midpoint, POS_EPS),
            total / 2.0,
            2e-13 * total.max(1.0)
        );
    }
}

#[test]
fn seg_fast_approx_bounding_box_contains_true_bounding_box_for_arcs() {
    for bulge in [-1.0, -0.25, 0.25, 1.0] {
        let (v1, v2) = arc_from_unit_circle(bulge);
        let exact = seg_bounding_box(v1, v2);
        let approx = seg_fast_approx_bounding_box(v1, v2);
        assert!(approx.min_x <= exact.min_x + 1e-14);
        assert!(approx.min_y <= exact.min_y + 1e-14);
        assert!(approx.max_x + 1e-14 >= exact.max_x);
        assert!(approx.max_y + 1e-14 >= exact.max_y);
    }
}

#[test]
fn seg_closest_point_on_arc_at_center_returns_start() {
    // Semicircle on x-axis from (0,0) to (4,0); center (2,0) is equidistant from endpoints
    // but the implementation returns v1 when the query is at the center.
    let v1 = PlineVertex::new(0.0, 0.0, 1.0);
    let v2 = PlineVertex::new(4.0, 0.0, 0.0);
    assert_fuzzy_eq!(
        seg_closest_point(v1, v2, Vector2::new(2.0, 0.0), 1e-9),
        v1.pos()
    );
}

#[test]
fn seg_distance_is_greater_than_at_arc_center_uses_endpoint_distance() {
    let v1 = PlineVertex::new(0.0, 0.0, 1.0);
    let v2 = PlineVertex::new(4.0, 0.0, 0.0);
    let center = Vector2::new(2.0, 0.0);
    // Start vertex (0, 0) is one radius (2) from center (2, 0) on this semicircle.
    let endpoint_distance = 2.0;
    assert!(!seg_distance_is_greater_than(
        v1,
        v2,
        center,
        endpoint_distance,
        1e-9
    ));
    assert!(seg_distance_is_greater_than(
        v1,
        v2,
        center,
        endpoint_distance - 1e-9,
        1e-9
    ));
}

#[test]
fn seg_length_half_circle_matches_pi_times_radius() {
    let v1 = PlineVertex::new(2.0, 2.0, 1.0);
    let v2 = PlineVertex::new(4.0, 2.0, 0.0);
    // Doc example: radius 1, sweep pi.
    assert_fuzzy_eq!(seg_length(v1, v2), PI);
}

#[test]
fn seg_length_full_turn_degenerate_arc_is_zero_length() {
    // Equal start/end with nonzero bulge collapses to zero path length.
    let vertex = PlineVertex::new(1.0, 2.0, 1.0);
    assert_fuzzy_eq!(seg_length(vertex, vertex), 0.0);
}

#[test]
fn seg_tangent_vector_on_unit_half_circle_matches_radial_perpendicular() {
    let bulge = 1.0;
    let (v1, v2) = arc_from_unit_circle(bulge);
    let point = point_on_unit_arc(bulge, 0.5);
    let tangent = seg_tangent_vector(v1, v2, point);
    // CCW unit circle tangent at angle theta is (-sin theta, cos theta) = point.perp().
    assert_fuzzy_eq!(tangent.normalize(), point.perp().normalize(), 2e-9);
}
