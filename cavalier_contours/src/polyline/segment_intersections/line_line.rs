use super::{SegmentIntersection, SegmentIntersections, resolve::resolve_curve_intersections};
use crate::{
    core::{
        math::{CurveIntersections, IntersectionPoint, dist_squared, intersect_lines},
        traits::Real,
    },
    polyline::{LineSegment, Segment},
};
use num_traits::real::Real as NumReal;

fn intersect_point_line<T>(
    point_segment: LineSegment<T>,
    line_segment: LineSegment<T>,
    point_is_seg1: bool,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let direction = line_segment.end - line_segment.start;
    let length_squared = direction.length_squared();
    if length_squared == T::zero() {
        let error_squared = dist_squared(point_segment.start, line_segment.start);
        if error_squared >= epsilon * epsilon {
            return SegmentIntersections::None;
        }
        return SegmentIntersections::One(SegmentIntersection {
            point: if point_is_seg1 {
                point_segment.start
            } else {
                line_segment.start
            },
            error_squared,
            seg1_t: T::zero(),
            seg2_t: T::zero(),
        });
    }

    let delta = point_segment.start - line_segment.start;
    let cross = direction.perp_dot(delta);
    let projection_error_squared = cross * cross / length_squared;
    if projection_error_squared >= epsilon * epsilon {
        return SegmentIntersections::None;
    }
    let t = delta.dot(direction) / length_squared;
    let projected_point = line_segment.start + direction.scale(t);
    let location = line_segment.locate(
        IntersectionPoint {
            point: projected_point,
            error_squared: projection_error_squared,
        },
        epsilon * epsilon,
    );
    let Some(location) = location else {
        return SegmentIntersections::None;
    };
    let hit = SegmentIntersection {
        point: location.point,
        error_squared: projection_error_squared + location.adjustment_error_squared,
        seg1_t: if point_is_seg1 { T::zero() } else { location.t },
        seg2_t: if point_is_seg1 { location.t } else { T::zero() },
    };
    SegmentIntersections::One(hit)
}

fn projected_boundary<T>(
    seg1: LineSegment<T>,
    seg2: LineSegment<T>,
    seg2_t: T,
) -> SegmentIntersection<T>
where
    T: Real,
{
    let point = seg2.point_at(seg2_t);
    let direction = seg1.end - seg1.start;
    let mut seg1_t = (point - seg1.start).dot(direction) / direction.length_squared();
    seg1_t = NumReal::max(T::zero(), NumReal::min(seg1_t, T::one()));
    let seg1_point = seg1.point_at(seg1_t);
    SegmentIntersection {
        point,
        error_squared: dist_squared(point, seg1_point),
        seg1_t,
        seg2_t,
    }
}

fn intersect_coincident_lines<T>(
    seg1: LineSegment<T>,
    seg2: LineSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let direction1 = seg1.end - seg1.start;
    let direction2 = seg2.end - seg2.start;
    let length1_squared = direction1.length_squared();
    let length2_squared = direction2.length_squared();
    let epsilon_squared = epsilon * epsilon;

    // A pair accepted as parallel at one point must remain position-equal over both finite spans.
    for (point, line_start, direction, length_squared) in [
        (seg1.start, seg2.start, direction2, length2_squared),
        (seg1.end, seg2.start, direction2, length2_squared),
        (seg2.start, seg1.start, direction1, length1_squared),
        (seg2.end, seg1.start, direction1, length1_squared),
    ] {
        let cross = direction.perp_dot(point - line_start);
        if cross * cross >= epsilon_squared * length_squared {
            for (point1, point2, seg1_t, seg2_t) in [
                (seg1.start, seg2.start, T::zero(), T::zero()),
                (seg1.start, seg2.end, T::zero(), T::one()),
                (seg1.end, seg2.start, T::one(), T::zero()),
                (seg1.end, seg2.end, T::one(), T::one()),
            ] {
                let error_squared = dist_squared(point1, point2);
                if error_squared < epsilon_squared {
                    return SegmentIntersections::One(SegmentIntersection {
                        point: point1,
                        error_squared,
                        seg1_t,
                        seg2_t,
                    });
                }
            }
            return SegmentIntersections::None;
        }
    }

    let mut t0 = seg2.parameter_at(seg1.start);
    let mut t1 = seg2.parameter_at(seg1.end);
    if t0 > t1 {
        std::mem::swap(&mut t0, &mut t1);
    }
    let seg2_length = length2_squared.sqrt();
    if !(t0 * seg2_length).fuzzy_lt_eps(seg2_length, epsilon)
        || !(t1 * seg2_length).fuzzy_gt_eps(T::zero(), epsilon)
    {
        return SegmentIntersections::None;
    }
    t0 = NumReal::max(T::zero(), NumReal::min(t0, T::one()));
    t1 = NumReal::max(T::zero(), NumReal::min(t1, T::one()));
    let start = projected_boundary(seg1, seg2, t0);
    let end = projected_boundary(seg1, seg2, t1);
    if dist_squared(start.point, end.point) < epsilon_squared {
        return SegmentIntersections::One(if start.error_squared <= end.error_squared {
            start
        } else {
            end
        });
    }
    SegmentIntersections::Overlap { start, end }
}

/// Intersects two finite directed line segments.
#[must_use]
#[inline(always)]
pub fn intersect_line_segments<T>(
    seg1: LineSegment<T>,
    seg2: LineSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    match (seg1.as_line(), seg2.as_line()) {
        (None, _) => intersect_point_line(seg1, seg2, true, epsilon),
        (_, None) => intersect_point_line(seg2, seg1, false, epsilon),
        (Some(line1), Some(line2)) => {
            let intersections = intersect_lines(line1, line2, epsilon);
            if matches!(intersections, CurveIntersections::Coincident) {
                intersect_coincident_lines(seg1, seg2, epsilon)
            } else {
                resolve_curve_intersections(
                    &Segment::Line(seg1),
                    &Segment::Line(seg2),
                    intersections,
                    epsilon,
                )
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::Vector2;

    #[test]
    fn overlap_boundaries_follow_second_segment() {
        let result = intersect_line_segments(
            LineSegment {
                start: Vector2::new(0.5, 0.0),
                end: Vector2::new(1.5, 0.0),
            },
            LineSegment {
                start: Vector2::new(1.0, 0.0),
                end: Vector2::zero(),
            },
            1e-5,
        );
        let SegmentIntersections::Overlap { start, end } = result else {
            panic!("expected overlap");
        };
        assert!(start.seg2_t < end.seg2_t);
        assert_eq!(start.point, Vector2::new(1.0, 0.0));
        assert_eq!(end.point, Vector2::new(0.5, 0.0));
    }

    #[test]
    fn nonparallel_intersection_has_both_parameters() {
        let result = intersect_line_segments(
            LineSegment {
                start: Vector2::zero(),
                end: Vector2::new(2.0, 0.0),
            },
            LineSegment {
                start: Vector2::new(1.0, -1.0),
                end: Vector2::new(1.0, 1.0),
            },
            1e-5,
        );
        let SegmentIntersections::One(hit) = result else {
            panic!("expected one intersection");
        };
        assert!((hit.seg1_t - 0.5).abs() < f64::EPSILON);
        assert!((hit.seg2_t - 0.5).abs() < f64::EPSILON);
    }
}
