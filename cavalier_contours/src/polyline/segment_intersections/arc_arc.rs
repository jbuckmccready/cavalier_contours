use super::{
    SegmentIntersection, SegmentIntersections,
    resolve::{resolve_curve_intersections, resolve_intersection},
};
use crate::{
    core::{
        math::{IntersectionPoint, Vector2, dist_squared, intersect_circles},
        traits::Real,
    },
    polyline::{ArcSegment, Segment},
};

fn endpoint_contact<T>(
    arc1: &ArcSegment<T>,
    point2: crate::core::math::Vector2<T>,
    seg2_t: T,
    epsilon_squared: T,
) -> Option<SegmentIntersection<T>>
where
    T: Real,
{
    let choices = [(arc1.start, T::zero()), (arc1.end, T::one())];
    let (point1, seg1_t) = choices
        .into_iter()
        .min_by(|a, b| dist_squared(a.0, point2).total_cmp(&dist_squared(b.0, point2)))
        .unwrap();
    let error_squared = dist_squared(point1, point2);
    (error_squared < epsilon_squared).then_some(SegmentIntersection {
        point: point2,
        error_squared,
        seg1_t,
        seg2_t,
    })
}

fn intersect_coincident_arcs<T>(
    arc1: &ArcSegment<T>,
    arc2: &ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let same_direction = arc1.sweep.is_sign_positive() == arc2.sweep.is_sign_positive();
    let arc1_start = if same_direction { arc1.start } else { arc1.end };
    let arc1_end = if same_direction { arc1.end } else { arc1.start };
    let interval_start = arc2.forward_angle(arc1_start);
    let sweep1 = arc1.sweep.abs();
    let sweep2 = arc2.sweep.abs();
    let epsilon_squared = epsilon * epsilon;

    for shift in [-T::tau(), T::zero(), T::tau()] {
        let shifted_start = interval_start + shift;
        let shifted_end = shifted_start + sweep1;
        let low = if shifted_start > T::zero() {
            shifted_start
        } else {
            T::zero()
        };
        let high = if shifted_end < sweep2 {
            shifted_end
        } else {
            sweep2
        };
        if low >= high {
            continue;
        }
        let source1 = if low == T::zero() {
            arc2.start
        } else {
            arc1_start
        };
        let source2 = if high == sweep2 { arc2.end } else { arc1_end };
        let project = |source: Vector2<T>| {
            let radial = source - arc2.center;
            arc2.center + radial.scale(arc2.radius / radial.length())
        };
        let point1 = project(source1);
        let point2 = project(source2);
        let candidate1 = IntersectionPoint {
            point: point1,
            error_squared: dist_squared(source1, point1),
        };
        let candidate2 = IntersectionPoint {
            point: point2,
            error_squared: dist_squared(source2, point2),
        };
        let seg1 = Segment::Arc(*arc1);
        let seg2 = Segment::Arc(*arc2);
        let (Some(start), Some(end)) = (
            resolve_intersection(&seg1, &seg2, candidate1, epsilon_squared),
            resolve_intersection(&seg1, &seg2, candidate2, epsilon_squared),
        ) else {
            continue;
        };
        if dist_squared(start.point, end.point) < epsilon_squared {
            return SegmentIntersections::One(if start.error_squared <= end.error_squared {
                start
            } else {
                end
            });
        }
        return SegmentIntersections::Overlap { start, end };
    }

    let start = endpoint_contact(arc1, arc2.start, T::zero(), epsilon_squared);
    let end = endpoint_contact(arc1, arc2.end, T::one(), epsilon_squared);
    match (start, end) {
        (None, None) => SegmentIntersections::None,
        (Some(hit), None) | (None, Some(hit)) => SegmentIntersections::One(hit),
        (Some(first), Some(second))
            if dist_squared(first.point, second.point) < epsilon_squared =>
        {
            SegmentIntersections::One(if first.error_squared <= second.error_squared {
                first
            } else {
                second
            })
        }
        (Some(first), Some(second)) => SegmentIntersections::Two([first, second]),
    }
}

/// Intersects two finite directed circular arcs.
#[must_use]
#[inline(always)]
pub fn intersect_arcs<T>(
    arc1: ArcSegment<T>,
    arc2: ArcSegment<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let intersections = intersect_circles(arc1.circle(), arc2.circle(), epsilon);
    if matches!(
        intersections,
        crate::core::math::CurveIntersections::Coincident
    ) {
        intersect_coincident_arcs(&arc1, &arc2, epsilon)
    } else {
        resolve_curve_intersections(
            &Segment::Arc(arc1),
            &Segment::Arc(arc2),
            intersections,
            epsilon,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::math::Vector2;

    fn arc(start: f64, sweep: f64) -> ArcSegment<f64> {
        ArcSegment {
            start: Vector2::new(start.cos(), start.sin()),
            end: Vector2::new((start + sweep).cos(), (start + sweep).sin()),
            center: Vector2::zero(),
            radius: 1.0,
            sweep,
        }
    }

    #[test]
    fn coincident_overlap_handles_angle_wrap() {
        let result = intersect_arcs(arc(6.0, 0.8), arc(0.1, 0.8), 1e-5);
        let SegmentIntersections::Overlap { start, end } = result else {
            panic!("expected overlap");
        };
        assert!(start.seg2_t < end.seg2_t);
    }

    #[test]
    fn coincident_endpoint_contact_uses_second_arc_point() {
        let first = arc(0.0, 0.5);
        let second = arc(0.5 + 5e-6, 0.5);
        let result = intersect_arcs(first, second, 1e-5);
        let SegmentIntersections::One(hit) = result else {
            panic!("expected endpoint contact");
        };
        assert_eq!(hit.point, second.start);
    }
}
