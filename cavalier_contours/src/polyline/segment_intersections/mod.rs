mod line_arc;
mod resolve;

use crate::core::{math::Vector2, traits::Real};

pub use line_arc::{intersect_arc_line, intersect_line_arc};

/// One resolved finite-segment intersection.
#[derive(Debug, Copy, Clone)]
pub struct SegmentIntersection<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub error_squared: T,
    pub seg1_t: T,
    pub seg2_t: T,
}

/// Result of intersecting two finite segments.
#[derive(Debug, Copy, Clone)]
pub enum SegmentIntersections<T>
where
    T: Real,
{
    None,
    Tangent(SegmentIntersection<T>),
    One(SegmentIntersection<T>),
    Two([SegmentIntersection<T>; 2]),
    Overlap {
        start: SegmentIntersection<T>,
        end: SegmentIntersection<T>,
    },
}

#[cfg(test)]
mod tests {
    use super::resolve::{resolve_curve_intersections, resolve_intersection};
    use super::*;
    use crate::{
        core::math::{CurveIntersections, IntersectionPoint},
        polyline::{LineSegment, Segment},
    };

    fn line(start: Vector2<f64>, end: Vector2<f64>) -> Segment<f64> {
        Segment::Line(LineSegment { start, end })
    }

    #[test]
    fn one_endpoint_snap_uses_exact_source_coordinate_and_parameters() {
        let epsilon = 1e-5;
        let seg1 = line(Vector2::zero(), Vector2::new(1.0, 0.0));
        let seg2 = line(
            Vector2::new(1.0 + epsilon / 2.0, -1.0),
            Vector2::new(1.0 + epsilon / 2.0, 1.0),
        );
        let hit = resolve_intersection(
            &seg1,
            &seg2,
            IntersectionPoint {
                point: Vector2::new(1.0 + epsilon / 2.0, 0.0),
                error_squared: 0.0,
            },
            epsilon * epsilon,
        )
        .unwrap();
        assert_eq!(hit.point, Vector2::new(1.0, 0.0));
        assert!((hit.seg1_t - 1.0).abs() < f64::EPSILON);
        assert!((hit.seg2_t - 0.5).abs() < f64::EPSILON);
    }

    #[test]
    fn two_hits_are_ordered_only_by_second_parameter() {
        let seg1 = line(Vector2::new(-2.0, 0.0), Vector2::new(2.0, 0.0));
        let seg2 = line(Vector2::new(2.0, 0.0), Vector2::new(-2.0, 0.0));
        let result = resolve_curve_intersections(
            &seg1,
            &seg2,
            CurveIntersections::Two([
                IntersectionPoint {
                    point: Vector2::new(-1.0, 0.0),
                    error_squared: 0.0,
                },
                IntersectionPoint {
                    point: Vector2::new(1.0, 0.0),
                    error_squared: 0.0,
                },
            ]),
            1e-5,
        );
        let SegmentIntersections::Two([first, second]) = result else {
            panic!("expected two intersections");
        };
        assert_eq!(first.point, Vector2::new(1.0, 0.0));
        assert_eq!(second.point, Vector2::new(-1.0, 0.0));
    }

    #[test]
    fn duplicate_hits_keep_the_smallest_final_error() {
        let epsilon = 1e-5;
        let seg1 = line(Vector2::zero(), Vector2::new(1.0, 0.0));
        let seg2 = line(Vector2::new(0.0, -1.0), Vector2::new(0.0, 1.0));
        let result = resolve_curve_intersections(
            &seg1,
            &seg2,
            CurveIntersections::Two([
                IntersectionPoint {
                    point: Vector2::zero(),
                    error_squared: 1.0,
                },
                IntersectionPoint {
                    point: Vector2::zero(),
                    error_squared: 0.0,
                },
            ]),
            epsilon,
        );
        let SegmentIntersections::One(hit) = result else {
            panic!("expected one intersection");
        };
        assert!(hit.error_squared.abs() < f64::EPSILON);
    }
}
