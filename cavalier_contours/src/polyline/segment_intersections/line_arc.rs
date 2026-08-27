use super::{SegmentIntersections, resolve::resolve_curve_intersections};
use crate::{
    core::{
        math::{CurveIntersections, IntersectionPoint, intersect_line_circle},
        traits::Real,
    },
    polyline::{ArcSegment, LineSegment, Segment},
};

fn line_circle_intersections<T>(
    line: LineSegment<T>,
    arc: ArcSegment<T>,
    epsilon: T,
) -> CurveIntersections<T>
where
    T: Real,
{
    let Some(whole_line) = line.as_line() else {
        let radial_error = (line.start - arc.center).length() - arc.radius;
        return if radial_error * radial_error < epsilon * epsilon {
            CurveIntersections::Tangent(IntersectionPoint {
                point: line.start,
                error_squared: radial_error * radial_error,
            })
        } else {
            CurveIntersections::None
        };
    };
    intersect_line_circle(whole_line, arc.circle(), epsilon)
}

/// Intersects a finite line segment and arc in line-arc argument order.
#[must_use]
pub fn intersect_line_arc<T>(
    line: LineSegment<T>,
    arc: ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let intersections = line_circle_intersections(line, arc, epsilon);
    resolve_curve_intersections(
        &Segment::Line(line),
        &Segment::Arc(arc),
        intersections,
        epsilon,
    )
}

/// Intersects a finite arc and line segment in arc-line argument order.
#[must_use]
pub fn intersect_arc_line<T>(
    arc: ArcSegment<T>,
    line: LineSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let intersections = line_circle_intersections(line, arc, epsilon);
    resolve_curve_intersections(
        &Segment::Arc(arc),
        &Segment::Line(line),
        intersections,
        epsilon,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::Vector2;

    fn upper_half_circle() -> ArcSegment<f64> {
        ArcSegment {
            start: Vector2::new(1.0, 0.0),
            end: Vector2::new(-1.0, 0.0),
            center: Vector2::zero(),
            radius: 1.0,
            sweep: std::f64::consts::PI,
        }
    }

    #[test]
    fn finite_line_and_arc_domains_clip_whole_curve_points() {
        let line = LineSegment {
            start: Vector2::new(-2.0, 0.5),
            end: Vector2::new(2.0, 0.5),
        };
        let result = intersect_line_arc(line, upper_half_circle(), 1e-5);
        let SegmentIntersections::Two([first, second]) = result else {
            panic!("expected two intersections");
        };
        assert!(first.seg2_t < second.seg2_t);
        assert!(first.point.x > second.point.x);
    }

    #[test]
    fn ordered_wrapper_resolves_parameters_in_argument_order() {
        let line = LineSegment {
            start: Vector2::new(-2.0, 0.5),
            end: Vector2::new(2.0, 0.5),
        };
        let SegmentIntersections::Two(line_arc) =
            intersect_line_arc(line, upper_half_circle(), 1e-5)
        else {
            panic!("expected two intersections");
        };
        let SegmentIntersections::Two(arc_line) =
            intersect_arc_line(upper_half_circle(), line, 1e-5)
        else {
            panic!("expected two intersections");
        };
        assert!((line_arc[0].seg1_t - arc_line[1].seg2_t).abs() < f64::EPSILON);
        assert!((line_arc[0].seg2_t - arc_line[1].seg1_t).abs() < f64::EPSILON);
    }

    #[test]
    fn point_line_uses_strict_circle_distance() {
        let epsilon = 1e-5;
        for (distance, intersects) in [(epsilon / 2.0, true), (epsilon, false)] {
            let point = Vector2::new(1.0 + distance, 0.0);
            let result = intersect_line_arc(
                LineSegment {
                    start: point,
                    end: point,
                },
                upper_half_circle(),
                epsilon,
            );
            assert_eq!(!matches!(result, SegmentIntersections::None), intersects);
        }
    }
}
