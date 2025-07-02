use crate::polyline::{
    FindIntersectsOptions, PlineContainmentOptions, PlineContainmentResult, PlineSource,
    internal::pline_intersects::scan_for_intersection,
};

use crate::core::{math::Vector2, traits::Real};

/// Determine if pline1 contains pline2.
///
/// Note that overlapping segments are considered intersections by this function.
///
/// Caution: Polylines with self-intersections may generate unexpected results.
/// Use scan_for_self_intersection() to find and reject self-intersecting polylines
/// if this is a possibility for your input data.
pub fn polyline_contains<P, R, T>(
    pline1: &P,
    pline2: &R,
    options: &PlineContainmentOptions<T>,
) -> PlineContainmentResult
where
    P: PlineSource<Num = T> + ?Sized,
    R: PlineSource<Num = T> + ?Sized,
    T: Real,
{
    if pline1.vertex_count() < 2
        || !pline1.is_closed()
        || pline2.vertex_count() < 2
        || !pline2.is_closed()
    {
        return PlineContainmentResult::InvalidInput;
    }
    let pos_equal_eps = options.pos_equal_eps;
    let constructed_index;
    let pline1_aabb_index = if let Some(x) = options.pline1_aabb_index {
        x
    } else {
        constructed_index = pline1.create_approx_aabb_index();
        &constructed_index
    };

    // helper functions to test if point is inside pline1 and pline2
    let point_in_pline1 = |point: Vector2<T>| pline1.winding_number(point) != 0;
    let point_in_pline2 = |point: Vector2<T>| pline2.winding_number(point) != 0;

    // helper functions (assuming no intersects between pline1 and pline2)
    let is_pline1_in_pline2 = || point_in_pline2(pline1.at(0).pos());
    let is_pline2_in_pline1 = || point_in_pline1(pline2.at(0).pos());

    if scan_for_intersection(
        pline1,
        pline2,
        &FindIntersectsOptions {
            pline1_aabb_index: Some(pline1_aabb_index),
            pos_equal_eps,
        },
    ) {
        PlineContainmentResult::Intersected
    } else if is_pline2_in_pline1() {
        PlineContainmentResult::Pline2InsidePline1
    } else if is_pline1_in_pline2() {
        PlineContainmentResult::Pline1InsidePline2
    } else {
        PlineContainmentResult::Disjoint
    }
}
