use super::{SegmentIntersection, SegmentIntersections};
use crate::{
    core::{
        math::{CurveIntersections, IntersectionPoint, dist_squared},
        traits::Real,
    },
    polyline::Segment,
};

pub(super) fn resolve_intersection<T>(
    seg1: &Segment<T>,
    seg2: &Segment<T>,
    candidate: IntersectionPoint<T>,
    epsilon_squared: T,
) -> Option<SegmentIntersection<T>>
where
    T: Real,
{
    let location1 = seg1.locate(candidate, epsilon_squared)?;
    let location2 = seg2.locate(candidate, epsilon_squared)?;
    let moved1 = location1.adjustment_error_squared != T::zero();
    let moved2 = location2.adjustment_error_squared != T::zero();

    let (point, adjustment_error_squared) = match (moved1, moved2) {
        (false, false) => (candidate.point, T::zero()),
        (true, false) => (location1.point, location1.adjustment_error_squared),
        (false, true) => (location2.point, location2.adjustment_error_squared),
        (true, true) if location1.point == location2.point => {
            (location1.point, location1.adjustment_error_squared)
        }
        (true, true) => {
            let separation_squared = dist_squared(location1.point, location2.point);
            if separation_squared >= epsilon_squared {
                return None;
            }
            if location1.adjustment_error_squared <= location2.adjustment_error_squared {
                (
                    location1.point,
                    location1.adjustment_error_squared + separation_squared,
                )
            } else {
                (
                    location2.point,
                    location2.adjustment_error_squared + separation_squared,
                )
            }
        }
    };

    Some(SegmentIntersection {
        point,
        error_squared: candidate.error_squared + adjustment_error_squared,
        seg1_t: location1.t,
        seg2_t: location2.t,
    })
}

pub(super) fn resolve_curve_intersections<T>(
    seg1: &Segment<T>,
    seg2: &Segment<T>,
    intersections: CurveIntersections<T>,
    epsilon: T,
) -> SegmentIntersections<T>
where
    T: Real,
{
    let epsilon_squared = epsilon * epsilon;
    match intersections {
        CurveIntersections::None | CurveIntersections::Coincident => SegmentIntersections::None,
        CurveIntersections::Tangent(candidate) => {
            resolve_intersection(seg1, seg2, candidate, epsilon_squared)
                .map_or(SegmentIntersections::None, SegmentIntersections::Tangent)
        }
        CurveIntersections::One(candidate) => {
            resolve_intersection(seg1, seg2, candidate, epsilon_squared)
                .map_or(SegmentIntersections::None, SegmentIntersections::One)
        }
        CurveIntersections::Two(candidates) => {
            let accepted = candidates
                .map(|candidate| resolve_intersection(seg1, seg2, candidate, epsilon_squared));
            match accepted {
                [None, None] => SegmentIntersections::None,
                [Some(hit), None] | [None, Some(hit)] => SegmentIntersections::One(hit),
                [Some(mut first), Some(mut second)] => {
                    if dist_squared(first.point, second.point) < epsilon_squared {
                        return SegmentIntersections::One(
                            if first.error_squared <= second.error_squared {
                                first
                            } else {
                                second
                            },
                        );
                    }
                    if second.seg2_t < first.seg2_t {
                        std::mem::swap(&mut first, &mut second);
                    }
                    SegmentIntersections::Two([first, second])
                }
            }
        }
    }
}
