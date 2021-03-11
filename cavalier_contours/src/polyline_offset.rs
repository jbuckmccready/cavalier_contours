use core::panic;
use std::collections::{BTreeMap, HashMap};

use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder};

use crate::{
    core_math::{
        angle, bulge_from_angle, delta_angle, dist_squared, point_from_parametric,
        point_within_arc_sweep, seg_arc_radius_and_center, seg_closest_point,
        seg_fast_approx_bounding_box, seg_midpoint, seg_split_at_point,
    },
    intersects::{
        circle_circle_intr, line_circle_intr, line_line_intr, pline_seg_intr, CircleCircleIntr,
        LineCircleIntr, LineLineIntr, PlineSegIntr,
    },
    polyline_intersects::{all_self_intersects_as_basic, find_intersects},
    PlineOffsetOptions, PlineVertex, Polyline, Real, Vector2,
};

#[derive(Debug, Copy, Clone)]
pub struct RawPlineOffsetSeg<T>
where
    T: Real,
{
    v1: PlineVertex<T>,
    v2: PlineVertex<T>,
    orig_v2_pos: Vector2<T>,
    collapsed_arc: bool,
}

pub fn create_untrimmed_raw_offset_segs<T>(
    polyline: &Polyline<T>,
    offset: T,
) -> Vec<RawPlineOffsetSeg<T>>
where
    T: Real,
{
    let mut result = Vec::new();
    let ln = polyline.len();
    if ln < 2 {
        return result;
    }

    let segment_count = if polyline.is_closed() {
        polyline.len()
    } else {
        polyline.len() - 1
    };
    result.reserve(segment_count);

    let process_line_seg = |v1: PlineVertex<T>, v2: PlineVertex<T>| -> RawPlineOffsetSeg<T> {
        let line_v = v2.pos() - v1.pos();
        let offset_v = line_v.unit_perp().scale(offset);
        RawPlineOffsetSeg {
            v1: PlineVertex::from_vector2(v1.pos() + offset_v, T::zero()),
            v2: PlineVertex::from_vector2(v2.pos() + offset_v, T::zero()),
            orig_v2_pos: v2.pos(),
            collapsed_arc: false,
        }
    };

    let process_arc_seg = |v1: PlineVertex<T>, v2: PlineVertex<T>| -> RawPlineOffsetSeg<T> {
        let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
        let offs = if v1.bulge_is_neg() { offset } else { -offset };
        let radius_after_offset = arc_radius + offs;
        let v1_to_center = (v1.pos() - arc_center).normalize();
        let v2_to_center = (v2.pos() - arc_center).normalize();

        let (new_v1_bulge, collapsed_arc) = if radius_after_offset.fuzzy_lt(T::zero()) {
            // collapsed arc, offset arc start and end points towards arc center and turn into line
            // handles case where offset vertexes are equal and simplifies path for clipping algorithm
            (T::zero(), true)
        } else {
            (v1.bulge, false)
        };

        RawPlineOffsetSeg {
            v1: PlineVertex::from_vector2(v1_to_center.scale(offs) + v1.pos(), new_v1_bulge),
            v2: PlineVertex::from_vector2(v2_to_center.scale(offs) + v2.pos(), v2.bulge),
            orig_v2_pos: v2.pos(),
            collapsed_arc,
        }
    };

    for (v1, v2) in polyline.iter_segments() {
        if v1.bulge_is_zero() {
            result.push(process_line_seg(v1, v2));
        } else {
            result.push(process_arc_seg(v1, v2));
        }
    }

    result
}

#[inline]
fn is_false_intersect<T>(t: T) -> bool
where
    T: Real,
{
    t < T::zero() || t > T::one()
}

fn bulge_for_connection<T>(
    arc_center: Vector2<T>,
    start_point: Vector2<T>,
    end_point: Vector2<T>,
    is_ccw: bool,
) -> T
where
    T: Real,
{
    let a1 = angle(arc_center, start_point);
    let a2 = angle(arc_center, end_point);
    let abs_sweep_angle = delta_angle(a1, a2).abs();
    let abs_bulge = bulge_from_angle(abs_sweep_angle);
    if is_ccw {
        abs_bulge
    } else {
        -abs_bulge
    }
}

fn connect_using_arc<T>(
    s1: &RawPlineOffsetSeg<T>,
    s2: &RawPlineOffsetSeg<T>,
    connection_arcs_ccw: bool,
    result: &mut Polyline<T>,
    pos_equal_eps: T,
) where
    T: Real,
{
    let arc_center = s1.orig_v2_pos;
    let sp = s1.v2.pos();
    let ep = s2.v1.pos();
    let bulge = bulge_for_connection(arc_center, sp, ep, connection_arcs_ccw);
    result.add_or_replace(sp.x, sp.y, bulge, pos_equal_eps);
    result.add_or_replace(ep.x, ep.y, s2.v1.bulge, pos_equal_eps);
}

fn line_line_join<T>(
    s1: &RawPlineOffsetSeg<T>,
    s2: &RawPlineOffsetSeg<T>,
    connection_arcs_ccw: bool,
    pos_equal_eps: T,
    result: &mut Polyline<T>,
) where
    T: Real,
{
    let v1 = &s1.v1;
    let v2 = &s1.v2;
    let u1 = &s2.v1;
    let u2 = &s2.v2;

    debug_assert!(
        v1.bulge_is_zero() && u1.bulge_is_zero(),
        "both segments should be lines"
    );

    if s1.collapsed_arc && s2.collapsed_arc {
        // connecting to/from collapsed arc, always connect using arc
        connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
    } else {
        match line_line_intr(v1.pos(), v2.pos(), u1.pos(), u2.pos()) {
            LineLineIntr::NoIntersect => {
                // just join with straight line
                result.add_or_replace(v2.x, v2.y, T::zero(), pos_equal_eps);
                result.add_or_replace(u1.x, u1.y, u1.bulge, pos_equal_eps);
            }
            LineLineIntr::TrueIntersect { seg1_t, .. } => {
                let intr_point = point_from_parametric(v1.pos(), v2.pos(), seg1_t);
                result.add_or_replace(intr_point.x, intr_point.y, T::zero(), pos_equal_eps);
            }
            LineLineIntr::Overlapping { .. } => {
                result.add_or_replace(v2.x, v2.y, T::zero(), pos_equal_eps);
            }
            LineLineIntr::FalseIntersect { seg1_t, seg2_t } => {
                if seg1_t > T::one() && is_false_intersect(seg2_t) {
                    // extend and join the lines together using arc
                    connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
                } else {
                    result.add_or_replace(v2.x, v2.y, T::zero(), pos_equal_eps);
                    result.add_or_replace(u1.x, u1.y, u1.bulge, pos_equal_eps);
                }
            }
        }
    }
}

fn line_arc_join<T>(
    s1: &RawPlineOffsetSeg<T>,
    s2: &RawPlineOffsetSeg<T>,
    connection_arcs_ccw: bool,
    pos_equal_eps: T,
    result: &mut Polyline<T>,
) where
    T: Real,
{
    let v1 = &s1.v1;
    let v2 = &s1.v2;
    let u1 = &s2.v1;
    let u2 = &s2.v2;

    debug_assert!(
        v1.bulge_is_zero() && !u1.bulge_is_zero(),
        "first segment should be line, second segment should be arc"
    );

    let (arc_radius, arc_center) = seg_arc_radius_and_center(*u1, *u2);

    let mut process_intersect = |t: T, intersect: Vector2<T>| {
        let true_line_intr = !is_false_intersect(t);
        let true_arc_intr =
            point_within_arc_sweep(arc_center, u1.pos(), u2.pos(), u1.bulge_is_neg(), intersect);

        if true_line_intr && true_arc_intr {
            // trim at intersect
            let a = angle(arc_center, intersect);
            let arc_end_angle = angle(arc_center, u2.pos());
            let theta = delta_angle(a, arc_end_angle);
            // ensure sign matches (may get flipped if intersect is at the very end of the arc,
            // in which case we do not want to update the bulge)
            if (theta > T::zero()) == u1.bulge_is_pos() {
                result.add_or_replace(
                    intersect.x,
                    intersect.y,
                    bulge_from_angle(theta),
                    pos_equal_eps,
                )
            } else {
                result.add_or_replace(intersect.x, intersect.y, u1.bulge, pos_equal_eps);
            }
            return;
        }

        if t > T::one() && !true_arc_intr {
            connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
            return;
        }

        if s1.collapsed_arc {
            connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
            return;
        }

        // connect using line
        result.add_or_replace(v2.x, v2.y, T::zero(), pos_equal_eps);
        result.add_or_replace_vertex(*u1, pos_equal_eps);
    };

    match line_circle_intr(v1.pos(), v2.pos(), arc_radius, arc_center) {
        LineCircleIntr::NoIntersect => {
            connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
        }
        LineCircleIntr::TangentIntersect { t0 } => {
            process_intersect(t0, point_from_parametric(v1.pos(), v2.pos(), t0));
        }
        LineCircleIntr::TwoIntersects { t0, t1 } => {
            // always use intersect closest to original point
            let intr1 = point_from_parametric(v1.pos(), v2.pos(), t0);
            let dist1 = dist_squared(intr1, s1.orig_v2_pos);
            let intr2 = point_from_parametric(v1.pos(), v2.pos(), t1);
            let dist2 = dist_squared(intr2, s1.orig_v2_pos);

            if dist1 < dist2 {
                process_intersect(t0, intr1);
            } else {
                process_intersect(t1, intr2);
            }
        }
    }
}

fn arc_line_join<T>(
    s1: &RawPlineOffsetSeg<T>,
    s2: &RawPlineOffsetSeg<T>,
    connection_arcs_ccw: bool,
    pos_equal_eps: T,
    result: &mut Polyline<T>,
) where
    T: Real,
{
    let v1 = &s1.v1;
    let v2 = &s1.v2;
    let u1 = &s2.v1;
    let u2 = &s2.v2;

    debug_assert!(
        !v1.bulge_is_zero() && u1.bulge_is_zero(),
        "first segment should be arc, second segment should be line"
    );

    let (arc_radius, arc_center) = seg_arc_radius_and_center(*v1, *v2);

    let mut process_intersect = |t: T, intersect: Vector2<T>| {
        let true_line_intr = !is_false_intersect(t);
        let true_arc_intr =
            point_within_arc_sweep(arc_center, v1.pos(), v2.pos(), v1.bulge_is_neg(), intersect);

        if true_line_intr && true_arc_intr {
            let prev_vertex = result.last().unwrap();
            if !prev_vertex.bulge_is_zero()
                && !prev_vertex.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps)
            {
                // modify previous bulge and trim at intersect
                let a = angle(arc_center, intersect);
                let (_, prev_arc_center) = seg_arc_radius_and_center(*prev_vertex, *v2);
                let prev_arc_start_angle = angle(prev_arc_center, prev_vertex.pos());
                let updated_prev_theta = delta_angle(prev_arc_start_angle, a);
                // ensure the sign matches (may get flipped if intersect is at the very end of the arc,
                // in which case we do not want to update the bulge)
                if (updated_prev_theta > T::zero()) == prev_vertex.bulge_is_pos() {
                    result.last_mut().unwrap().bulge = bulge_from_angle(updated_prev_theta);
                }
            }

            result.add_or_replace(intersect.x, intersect.y, T::zero(), pos_equal_eps);
            return;
        }

        connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
    };

    match line_circle_intr(u1.pos(), u2.pos(), arc_radius, arc_center) {
        LineCircleIntr::NoIntersect => {
            connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
        }
        LineCircleIntr::TangentIntersect { t0 } => {
            process_intersect(t0, point_from_parametric(u1.pos(), u2.pos(), t0));
        }
        LineCircleIntr::TwoIntersects { t0, t1 } => {
            // always use intersect closest to original point
            let orig_point = if s2.collapsed_arc {
                u1.pos()
            } else {
                s1.orig_v2_pos
            };
            let intr1 = point_from_parametric(u1.pos(), u2.pos(), t0);
            let dist1 = dist_squared(intr1, orig_point);
            let intr2 = point_from_parametric(u1.pos(), u2.pos(), t1);
            let dist2 = dist_squared(intr2, orig_point);

            if dist1 < dist2 {
                process_intersect(t0, intr1);
            } else {
                process_intersect(t1, intr2);
            }
        }
    }
}

fn arc_arc_join<T>(
    s1: &RawPlineOffsetSeg<T>,
    s2: &RawPlineOffsetSeg<T>,
    connection_arcs_ccw: bool,
    pos_equal_eps: T,
    result: &mut Polyline<T>,
) where
    T: Real,
{
    let v1 = &s1.v1;
    let v2 = &s1.v2;
    let u1 = &s2.v1;
    let u2 = &s2.v2;

    debug_assert!(
        !v1.bulge_is_zero() && !u1.bulge_is_zero(),
        "both segments should be arcs"
    );

    let (arc1_radius, arc1_center) = seg_arc_radius_and_center(*v1, *v2);
    let (arc2_radius, arc2_center) = seg_arc_radius_and_center(*u1, *u2);

    let mut process_intersect = |intersect: Vector2<T>| {
        let true_arc_intr1 = point_within_arc_sweep(
            arc1_center,
            v1.pos(),
            v2.pos(),
            v1.bulge_is_neg(),
            intersect,
        );
        let true_arc_intr2 = point_within_arc_sweep(
            arc2_center,
            u1.pos(),
            u2.pos(),
            u1.bulge_is_neg(),
            intersect,
        );

        if true_arc_intr1 && true_arc_intr2 {
            let prev_vertex = result.last().unwrap();

            if !prev_vertex.bulge_is_zero()
                && !prev_vertex.pos().fuzzy_eq_eps(v2.pos(), pos_equal_eps)
            {
                // modify previous bulge and trim at intersect
                let a1 = angle(arc1_center, intersect);
                let (_, prev_arc_center) = seg_arc_radius_and_center(*prev_vertex, *v2);
                let prev_arc_start_angle = angle(prev_arc_center, prev_vertex.pos());
                let updated_prev_theta = delta_angle(prev_arc_start_angle, a1);
                // ensure the sign matches (may get flipped if intersect is at the very end of the arc,
                // in which case we do not want to update the bulge)
                if (updated_prev_theta > T::zero()) == prev_vertex.bulge_is_pos() {
                    result.last_mut().unwrap().bulge = bulge_from_angle(updated_prev_theta);
                }
            }

            // add the vertex at our current trim/join point
            let a2 = angle(arc2_center, intersect);
            let end_angle = angle(arc2_center, u2.pos());
            let theta = delta_angle(a2, end_angle);

            // again ensure sign matches before updating bulge
            if (theta > T::zero()) == u1.bulge_is_pos() {
                result.add_or_replace(
                    intersect.x,
                    intersect.y,
                    bulge_from_angle(theta),
                    pos_equal_eps,
                );
            } else {
                result.add_or_replace(intersect.x, intersect.y, u1.bulge, pos_equal_eps);
            }

            return;
        }

        connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
    };

    match circle_circle_intr(arc1_radius, arc1_center, arc2_radius, arc2_center) {
        CircleCircleIntr::NoIntersect => {
            connect_using_arc(s1, s2, connection_arcs_ccw, result, pos_equal_eps);
        }
        CircleCircleIntr::TangentIntersect { point } => {
            process_intersect(point);
        }
        CircleCircleIntr::TwoIntersects { point1, point2 } => {
            // always use intersect closest to original point
            let dist1 = dist_squared(point1, s1.orig_v2_pos);
            let dist2 = dist_squared(point2, s1.orig_v2_pos);
            if dist1 < dist2 {
                process_intersect(point1);
            } else {
                process_intersect(point2);
            }
        }
        CircleCircleIntr::Overlapping => {
            // same arc radius and center, just add the vertex (nothing to trim/extend)
            result.add_or_replace_vertex(*u1, pos_equal_eps);
        }
    }
}

pub fn create_raw_offset_polyline<T>(
    polyline: &Polyline<T>,
    offset: T,
    pos_equal_eps: T,
) -> Polyline<T>
where
    T: Real,
{
    if polyline.len() < 2 {
        return Polyline::new();
    }

    let raw_offset_segs = create_untrimmed_raw_offset_segs(polyline, offset);
    if raw_offset_segs.len() == 0 {
        return Polyline::new();
    }

    // detect single collapsed arc segment
    if raw_offset_segs.len() == 1 && raw_offset_segs[0].collapsed_arc {
        return Polyline::new();
    }

    let connection_arcs_ccw = offset < T::zero();

    let join_seg_pair =
        |s1: &RawPlineOffsetSeg<T>, s2: &RawPlineOffsetSeg<T>, result: &mut Polyline<T>| {
            let s1_is_line = s1.v1.bulge_is_zero();
            let s2_is_line = s2.v1.bulge_is_zero();
            match (s1_is_line, s2_is_line) {
                (true, true) => line_line_join(s1, s2, connection_arcs_ccw, pos_equal_eps, result),
                (true, false) => line_arc_join(s1, s2, connection_arcs_ccw, pos_equal_eps, result),
                (false, true) => arc_line_join(s1, s2, connection_arcs_ccw, pos_equal_eps, result),
                (false, false) => arc_arc_join(s1, s2, connection_arcs_ccw, pos_equal_eps, result),
            }
        };

    let mut result = Polyline::with_capacity(polyline.len());
    result.set_is_closed(polyline.is_closed());

    // add the very first vertex
    result.add_vertex(raw_offset_segs.first().unwrap().v1);

    // join first two segments and determine if first vertex was replaced (to know how to handle
    // last two segment joins for closed polyline)
    let mut offset_seg_pairs = raw_offset_segs.windows(2);
    if let Some([s1, s2]) = offset_seg_pairs.next() {
        join_seg_pair(s1, s2, &mut result);
    }

    let first_vertex_replaced = result.len() == 1;

    while let Some([s1, s2]) = offset_seg_pairs.next() {
        join_seg_pair(s1, s2, &mut result);
    }

    if polyline.is_closed() && result.len() > 1 {
        // join closing segments at vertex indexes (n, 0) and (0, 1)
        let s1 = &raw_offset_segs.last().unwrap();
        let s2 = &raw_offset_segs[0];

        // temp polyline to capture results of joining (to avoid mutating result)
        let mut closing_part_result = Polyline::new();
        closing_part_result.add_vertex(*result.last().unwrap());
        join_seg_pair(s1, s2, &mut closing_part_result);

        // update last vertexes
        *result.last_mut().unwrap() = closing_part_result[0];
        for v in closing_part_result.iter().skip(1) {
            result.add_vertex(*v);
        }

        // update first vertex (only if it has not already been updated/replaced)
        if !first_vertex_replaced {
            let updated_first_pos = closing_part_result.last().unwrap().pos();
            if result[0].bulge_is_zero() {
                // just update position
                result[0].x = updated_first_pos.x;
                result[0].y = updated_first_pos.y;
            } else if result.len() > 1 {
                // update position and bulge
                let (_, arc_center) = seg_arc_radius_and_center(result[0], result[1]);
                let a1 = angle(arc_center, updated_first_pos);
                let a2 = angle(arc_center, result[1].pos());
                let updated_theta = delta_angle(a1, a2);
                if (updated_theta < T::zero() && result[0].bulge_is_pos())
                    || (updated_theta > T::zero() && result[0].bulge_is_neg())
                {
                    // first vertex not valid, just update its position (it will be removed later)
                    result[0].x = updated_first_pos.x;
                    result[0].y = updated_first_pos.y;
                } else {
                    // update position and bulge
                    result.set_vertex(
                        0,
                        updated_first_pos.x,
                        updated_first_pos.y,
                        bulge_from_angle(updated_theta),
                    );
                }
            }
        }

        // must do final singularity prune between last, first, and second vertex because after joining segments,
        // (n, 0) and (0, 1) they may have been introduced
        if result.len() > 1 {
            if result[0]
                .pos()
                .fuzzy_eq_eps(result.last().unwrap().pos(), pos_equal_eps)
            {
                result.remove_last();
            }

            if result[0].pos().fuzzy_eq_eps(result[1].pos(), pos_equal_eps) {
                result.remove(0);
            }
        }
    } else {
        // not closed polyline or less than 2 vertexes
        let last_raw_offset_vertex = raw_offset_segs.last().unwrap().v2;
        result.add_or_replace_vertex(last_raw_offset_vertex, pos_equal_eps);
    }

    // if due to joining of segments we are left with only 1 vertex then return empty polyline
    if result.len() == 1 {
        result.clear();
    }

    result
}

fn point_valid_for_offset<T>(
    polyline: &Polyline<T>,
    offset: T,
    spatial_index: &StaticAABB2DIndex<T>,
    point: Vector2<T>,
    query_stack: &mut Vec<usize>,
    offset_tol: T,
) -> bool
where
    T: Real,
{
    let abs_offset = offset.abs() - offset_tol;
    let min_dist = abs_offset * abs_offset;
    let mut point_valid = true;
    let mut visitor = |i: usize| -> bool {
        let j = polyline.next_wrapping_index(i);
        let closest_point = seg_closest_point(polyline[i], polyline[j], point);
        let dist = dist_squared(closest_point, point);
        point_valid = dist > min_dist;
        point_valid
    };

    spatial_index.visit_query_with_stack(
        point.x - abs_offset,
        point.y - abs_offset,
        point.x + abs_offset,
        point.y + abs_offset,
        &mut visitor,
        query_stack,
    );
    point_valid
}

#[derive(Debug, Clone)]
pub struct OpenPolylineSlice<T> {
    intr_start_index: usize,
    polyline: Polyline<T>,
}

impl<T> OpenPolylineSlice<T> {
    pub fn new(intr_start_index: usize, slice: Polyline<T>) -> Self {
        OpenPolylineSlice {
            intr_start_index,
            polyline: slice,
        }
    }
}

pub fn slices_from_raw_offset<T>(
    original_polyline: &Polyline<T>,
    raw_offset_polyline: &Polyline<T>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> Vec<OpenPolylineSlice<T>>
where
    T: Real,
{
    let mut result = Vec::new();
    if raw_offset_polyline.len() < 2 {
        return result;
    }

    let pos_equal_eps = options.pos_equal_eps;
    let offset_dist_eps = options.offset_dist_eps;

    let raw_offset_index = raw_offset_polyline.create_approx_spatial_index().unwrap();
    let self_intrs =
        all_self_intersects_as_basic(raw_offset_polyline, &raw_offset_index, pos_equal_eps);

    dbg!(raw_offset_polyline.len());
    dbg!(raw_offset_polyline);
    // dbg!(&self_intrs);
    let mut query_stack = Vec::new();
    if self_intrs.len() == 0 {
        // no self intersects, test point on polyline is valid
        if !point_valid_for_offset(
            &original_polyline,
            offset,
            &orig_polyline_index,
            raw_offset_polyline[0].pos(),
            &mut query_stack,
            offset_dist_eps,
        ) {
            // not valid
            return result;
        }

        // is valid, copy and convert raw offset into open slice
        let mut slice = raw_offset_polyline.clone();
        slice.set_is_closed(false);
        let mut first_vertex = raw_offset_polyline[0];
        first_vertex.bulge = T::zero();
        slice.add_vertex(first_vertex);
        result.push(OpenPolylineSlice::new(usize::max_value(), slice));
        return result;
    }

    // using HashMap rather than BTreeMap for performance (as is used in
    // dualSliceAtIntersectsForOffset) since all slices will stitch together to form closed
    // loops so later when slices are stitched together the order that slices are visited
    // does not matter
    let mut intersects_lookup =
        HashMap::<usize, Vec<Vector2<T>>>::with_capacity(2 * self_intrs.len());

    for si in &self_intrs {
        intersects_lookup
            .entry(si.start_index1)
            .or_default()
            .push(si.point);
        intersects_lookup
            .entry(si.start_index2)
            .or_default()
            .push(si.point);
    }

    // sort intersects by distance from segment start vertex
    for (&i, intr_list) in intersects_lookup.iter_mut() {
        let start_pos = raw_offset_polyline[i].pos();
        intr_list.sort_unstable_by(|&si1, &si2| {
            let dist1 = dist_squared(si1, start_pos);
            let dist2 = dist_squared(si2, start_pos);
            dist1.partial_cmp(&dist2).unwrap()
        });
    }

    let intersects_original_pline =
        |v1: PlineVertex<T>, v2: PlineVertex<T>, query_stack: &mut Vec<usize>| -> bool {
            let approx_bb = seg_fast_approx_bounding_box(v1, v2);
            let mut has_intersect = false;
            let mut visitor = |i: usize| -> bool {
                let j = original_polyline.next_wrapping_index(i);
                has_intersect = !matches!(
                    pline_seg_intr(v1, v2, original_polyline[i], original_polyline[j]),
                    PlineSegIntr::NoIntersect
                );
                !has_intersect
            };

            let fuzz = T::fuzzy_epsilon();
            orig_polyline_index.visit_query_with_stack(
                approx_bb.min_x - fuzz,
                approx_bb.min_y - fuzz,
                approx_bb.max_x + fuzz,
                approx_bb.max_y + fuzz,
                &mut visitor,
                query_stack,
            );
            has_intersect
        };

    let point_valid_dist = |point: Vector2<T>, query_stack: &mut Vec<usize>| -> bool {
        point_valid_for_offset(
            original_polyline,
            offset,
            &orig_polyline_index,
            point,
            query_stack,
            offset_dist_eps,
        )
    };

    for (&start_index, intr_list) in intersects_lookup.iter() {
        let next_index = raw_offset_polyline.next_wrapping_index(start_index);
        let start_vertex = raw_offset_polyline[start_index];
        let end_vertex = raw_offset_polyline[next_index];
        if intr_list.len() != 1 {
            // build all the slices between the N intersects in intr_list (N > 1), skipping the first
            // slice (to be processed at the end)
            let first_split =
                seg_split_at_point(start_vertex, end_vertex, intr_list[0], pos_equal_eps);
            let mut prev_vertex = first_split.split_vertex;
            for i in 1..intr_list.len() {
                let split =
                    seg_split_at_point(prev_vertex, end_vertex, intr_list[i], pos_equal_eps);
                // update prev_vertex for next loop iteration
                prev_vertex = split.split_vertex;
                // skip if positions overlap
                if split
                    .updated_start
                    .pos()
                    .fuzzy_eq_eps(split.split_vertex.pos(), pos_equal_eps)
                {
                    continue;
                }

                // test start point
                if !point_valid_dist(split.updated_start.pos(), &mut query_stack) {
                    continue;
                }

                // test end point
                if !point_valid_dist(split.split_vertex.pos(), &mut query_stack) {
                    continue;
                }

                // test segment midpoint
                let midpoint = seg_midpoint(split.updated_start, split.split_vertex);
                if !point_valid_dist(midpoint, &mut query_stack) {
                    continue;
                }

                // test intersection with original polyline
                if intersects_original_pline(
                    split.updated_start,
                    split.split_vertex,
                    &mut query_stack,
                ) {
                    continue;
                }

                // passed all tests, add the slice
                let mut slice = Polyline::new();
                slice.add_vertex(split.updated_start);
                slice.add_vertex(split.split_vertex);
                result.push(OpenPolylineSlice::new(start_index, slice));
            }
        }

        // build the slice between the last intersect in the intr_list and the next intersect found
        // check that the first point is valid
        let slice_start_point = *intr_list.last().unwrap();
        if !point_valid_dist(slice_start_point, &mut query_stack) {
            continue;
        }

        let split = seg_split_at_point(start_vertex, end_vertex, slice_start_point, pos_equal_eps);
        let mut slice = Polyline::new();
        slice.add_vertex(split.split_vertex);

        let mut index = next_index;
        let mut is_valid_pline = true;
        let mut loop_count = 0;
        let max_loop_count = raw_offset_polyline.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!("loop_count exceeded max_loop_count while creating slices from raw offset");
            }
            loop_count += 1;

            let current_vertex = raw_offset_polyline[index];
            // check that vertex point is valid
            if !point_valid_dist(current_vertex.pos(), &mut query_stack) {
                is_valid_pline = false;
                break;
            }

            // check that the segment does not intersect original polyline
            if intersects_original_pline(*slice.last().unwrap(), current_vertex, &mut query_stack) {
                is_valid_pline = false;
                break;
            }

            // add vertex
            slice.add_or_replace_vertex(current_vertex, pos_equal_eps);

            // check if segment that starts at current vertex just added to slice has an intersect
            if let Some(next_intr_list) = intersects_lookup.get(&index) {
                // there is an intersect, slice is done, check if final segment is valid

                // check intersect point is valid (which will be the end of the slice)
                let intersect_point = next_intr_list[0];
                if !point_valid_dist(intersect_point, &mut query_stack) {
                    is_valid_pline = false;
                    break;
                }

                let next_index = raw_offset_polyline.next_wrapping_index(index);
                let split = seg_split_at_point(
                    current_vertex,
                    raw_offset_polyline[next_index],
                    intersect_point,
                    pos_equal_eps,
                );

                let slice_end_vertex = PlineVertex::from_vector2(intersect_point, T::zero());
                // check midpoint is valid
                let midpoint = seg_midpoint(split.updated_start, slice_end_vertex);
                if !point_valid_dist(midpoint, &mut query_stack) {
                    is_valid_pline = false;
                    break;
                }

                // trim last added vertex and add final intersect point
                *slice.last_mut().unwrap() = split.updated_start;
                slice.add_or_replace_vertex(slice_end_vertex, pos_equal_eps);
                break;
            }
            // else there is not an intersect, increment index and continue
            index = raw_offset_polyline.next_wrapping_index(index);
        }

        is_valid_pline = is_valid_pline && slice.len() > 1;

        if is_valid_pline && slice[0].pos().fuzzy_eq(slice.last().unwrap().pos()) {
            // discard very short slice loops (invalid loops may arise due to valid offset distance
            // threshold)
            is_valid_pline = slice.path_length() > T::from(1e-2).unwrap();
        }

        if is_valid_pline {
            result.push(OpenPolylineSlice::new(start_index, slice));
        }
    }

    result
}

/// Adds circle intersects to the `intersect_lookups` given.
fn visit_circle_intersects<T, F>(
    pline: &Polyline<T>,
    circle_center: Vector2<T>,
    circle_radius: T,
    spatial_index: &StaticAABB2DIndex<T>,
    visitor: &mut F,
    options: &PlineOffsetOptions<T>,
) where
    T: Real,
    F: FnMut(usize, Vector2<T>),
{
    let pos_equal_eps = options.pos_equal_eps;

    let is_valid_line_intr = |t: T| -> bool {
        // skip false intersects and intersects at start of seg
        !is_false_intersect(t) && t.abs() > pos_equal_eps
    };

    let is_valid_arc_intr = |arc_center: Vector2<T>,
                             arc_start: Vector2<T>,
                             arc_end: Vector2<T>,
                             bulge: T,
                             intr: Vector2<T>|
     -> bool {
        // skip false intersects and intersects at start of seg
        !arc_start.fuzzy_eq_eps(intr, pos_equal_eps)
            && point_within_arc_sweep(arc_center, arc_start, arc_end, bulge < T::zero(), intr)
    };

    let query_results = spatial_index.query(
        circle_center.x - circle_radius,
        circle_center.y - circle_radius,
        circle_center.x + circle_radius,
        circle_center.y + circle_radius,
    );

    for start_index in query_results {
        let v1 = pline[start_index];
        let v2 = pline[pline.next_wrapping_index(start_index)];
        if v1.bulge_is_zero() {
            match line_circle_intr(v1.pos(), v2.pos(), circle_radius, circle_center) {
                LineCircleIntr::NoIntersect => {}
                LineCircleIntr::TangentIntersect { t0 } => {
                    if is_valid_line_intr(t0) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t0));
                    }
                }
                LineCircleIntr::TwoIntersects { t0, t1 } => {
                    if is_valid_line_intr(t0) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t0));
                    }
                    if is_valid_line_intr(t1) {
                        visitor(start_index, point_from_parametric(v1.pos(), v2.pos(), t1));
                    }
                }
            }
        } else {
            let (arc_radius, arc_center) = seg_arc_radius_and_center(v1, v2);
            match circle_circle_intr(arc_radius, arc_center, circle_radius, circle_center) {
                CircleCircleIntr::NoIntersect => {}
                CircleCircleIntr::TangentIntersect { point } => {
                    if is_valid_arc_intr(arc_center, v1.pos(), v2.pos(), v1.bulge, point) {
                        visitor(start_index, point);
                    }
                }
                CircleCircleIntr::TwoIntersects { point1, point2 } => {
                    if is_valid_arc_intr(arc_center, v1.pos(), v2.pos(), v1.bulge, point1) {
                        visitor(start_index, point1);
                    }
                    if is_valid_arc_intr(arc_center, v2.pos(), v2.pos(), v2.bulge, point2) {
                        visitor(start_index, point2);
                    }
                }
                CircleCircleIntr::Overlapping => {}
            }
        }
    }
}

pub fn slices_from_dual_raw_offsets<T>(
    original_polyline: &Polyline<T>,
    raw_offset_polyline: &Polyline<T>,
    dual_raw_offset_polyline: &Polyline<T>,
    orig_polyline_index: &StaticAABB2DIndex<T>,
    offset: T,
    options: &PlineOffsetOptions<T>,
) -> Vec<OpenPolylineSlice<T>>
where
    T: Real,
{
    let mut result = Vec::new();
    if raw_offset_polyline.len() < 2 {
        return result;
    }

    let pos_equal_eps = options.pos_equal_eps;
    let offset_dist_eps = options.offset_dist_eps;

    let raw_offset_index = raw_offset_polyline.create_approx_spatial_index().unwrap();

    let self_intrs =
        all_self_intersects_as_basic(raw_offset_polyline, &raw_offset_index, pos_equal_eps);

    let dual_intrs = find_intersects(
        raw_offset_polyline,
        dual_raw_offset_polyline,
        &raw_offset_index,
    );

    // using BTreeMap rather than  HashMap since we want to construct the slices in vertex index order
    // and we do so by looping through all intersects (required later when slices are stitched
    // together, because slices may not all form closed loops/polylines so must go in order of
    // indexes to ensure longest stitched results are formed)
    let mut intersects_lookup = BTreeMap::<usize, Vec<Vector2<T>>>::new();

    // helper function to add intersects to the lookup
    let mut add_intr = |start_index: usize, intr: Vector2<T>| {
        intersects_lookup.entry(start_index).or_default().push(intr);
    };

    if !original_polyline.is_closed() {
        // add intersects between circles generated at original open polyline end points and raw offset polyline
        let circle_radius = offset.abs();
        visit_circle_intersects(
            raw_offset_polyline,
            original_polyline[0].pos(),
            circle_radius,
            &raw_offset_index,
            &mut add_intr,
            &options,
        );
        visit_circle_intersects(
            raw_offset_polyline,
            original_polyline.last().unwrap().pos(),
            circle_radius,
            &raw_offset_index,
            &mut add_intr,
            &options,
        );
    }

    // add all self intersects
    for &si in self_intrs.iter() {
        add_intr(si.start_index1, si.point);
        add_intr(si.start_index2, si.point);
    }

    // only add intersects with start_index1 from dual intersects (corresponds to the the raw offset polyline)
    for &intr in dual_intrs.basic_intersects.iter() {
        add_intr(intr.start_index1, intr.point);
    }
    for &intr in dual_intrs.overlapping_intersects.iter() {
        add_intr(intr.start_index1, intr.point1);
        add_intr(intr.start_index1, intr.point2);
    }

    let mut query_stack = Vec::with_capacity(8);

    if intersects_lookup.is_empty() {
        // test a point on raw offset polyline
        if !point_valid_for_offset(
            original_polyline,
            offset,
            &orig_polyline_index,
            raw_offset_polyline[0].pos(),
            &mut query_stack,
            offset_dist_eps,
        ) {
            return result;
        }

        // copy and convert raw offset into open polyline
        let mut pline = raw_offset_polyline.clone();
        pline.set_is_closed(false);
        if original_polyline.is_closed() {
            let first_vert = pline[0];
            pline.add(first_vert.x, first_vert.y, T::zero());
        }
        result.push(OpenPolylineSlice::new(usize::MAX, pline));
        return result;
    }

    // sort intersects by distance from segment start vertex
    for (&i, intr_list) in intersects_lookup.iter_mut() {
        let start_pos = raw_offset_polyline[i].pos();
        intr_list.sort_unstable_by(|&si1, &si2| {
            let dist1 = dist_squared(si1, start_pos);
            let dist2 = dist_squared(si2, start_pos);
            dist1.partial_cmp(&dist2).unwrap()
        });
    }

    let intersects_original_pline =
        |v1: PlineVertex<T>, v2: PlineVertex<T>, query_stack: &mut Vec<usize>| -> bool {
            let approx_bb = seg_fast_approx_bounding_box(v1, v2);
            let mut has_intersect = false;
            let mut visitor = |i: usize| -> bool {
                let j = original_polyline.next_wrapping_index(i);
                has_intersect = !matches!(
                    pline_seg_intr(v1, v2, original_polyline[i], original_polyline[j]),
                    PlineSegIntr::NoIntersect
                );
                !has_intersect
            };

            let fuzz = T::fuzzy_epsilon();
            orig_polyline_index.visit_query_with_stack(
                approx_bb.min_x - fuzz,
                approx_bb.min_y - fuzz,
                approx_bb.max_x + fuzz,
                approx_bb.max_y + fuzz,
                &mut visitor,
                query_stack,
            );
            has_intersect
        };

    let point_valid_dist = |point: Vector2<T>, query_stack: &mut Vec<usize>| -> bool {
        point_valid_for_offset(
            original_polyline,
            offset,
            &orig_polyline_index,
            point,
            query_stack,
            offset_dist_eps,
        )
    };

    if !original_polyline.is_closed() {
        // build first slice that ends at the first intersect since we will not wrap back to
        // capture it as in the case of a closed polyline
        let mut first_slice_pline = Polyline::new();
        let mut index = 0;
        let mut loop_count = 0;
        let max_loop_count = raw_offset_polyline.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!("loop_count exceeded max_loop_count while creating slices from raw offset");
            }
            loop_count += 1;

            if let Some(intr_list) = intersects_lookup.get(&index) {
                // intersect found, test segment will be valid before adding first slice
                let intr_pos = intr_list[0];
                if !point_valid_dist(intr_pos, &mut query_stack) {
                    break;
                }

                let split = seg_split_at_point(
                    raw_offset_polyline[index],
                    raw_offset_polyline[index + 1],
                    intr_pos,
                    pos_equal_eps,
                );

                let slice_end_vertex = PlineVertex::new(intr_pos.x, intr_pos.y, T::zero());
                let midpoint = seg_midpoint(split.updated_start, slice_end_vertex);
                if !point_valid_dist(midpoint, &mut query_stack) {
                    break;
                }

                if intersects_original_pline(
                    split.updated_start,
                    slice_end_vertex,
                    &mut query_stack,
                ) {
                    break;
                }

                first_slice_pline.add_or_replace_vertex(split.updated_start, pos_equal_eps);
                first_slice_pline.add_or_replace_vertex(slice_end_vertex, pos_equal_eps);
                // start index always 0 since this is the first slice
                result.push(OpenPolylineSlice::new(0, first_slice_pline));
                break;
            } else {
                //no intersect found, test segment will be valid before adding vertex
                if !point_valid_dist(raw_offset_polyline[index].pos(), &mut query_stack) {
                    break;
                }

                // index check (only test segment if we're not adding the first vertex)
                if index != 0
                    && intersects_original_pline(
                        *first_slice_pline.last().unwrap(),
                        raw_offset_polyline[index],
                        &mut query_stack,
                    )
                {
                    break;
                }

                first_slice_pline.add_or_replace_vertex(raw_offset_polyline[index], pos_equal_eps);
            }

            index += 1;
        }
    }

    for (&start_index, intr_list) in intersects_lookup.iter() {
        let next_index = raw_offset_polyline.next_wrapping_index(start_index);
        let start_vertex = raw_offset_polyline[start_index];
        let end_vertex = raw_offset_polyline[next_index];
        if intr_list.len() != 1 {
            // build all the slices between the N intersects in intr_list (N > 1), skipping the first
            // slice (to be processed at the end)
            let first_split =
                seg_split_at_point(start_vertex, end_vertex, intr_list[0], pos_equal_eps);
            let mut prev_vertex = first_split.split_vertex;
            for i in 1..intr_list.len() {
                let split =
                    seg_split_at_point(prev_vertex, end_vertex, intr_list[i], pos_equal_eps);
                // update prev_vertex for next loop iteration
                prev_vertex = split.split_vertex;
                // skip if positions overlap
                if split
                    .updated_start
                    .pos()
                    .fuzzy_eq_eps(split.split_vertex.pos(), pos_equal_eps)
                {
                    continue;
                }

                // test start point
                if !point_valid_dist(split.updated_start.pos(), &mut query_stack) {
                    continue;
                }

                // test end point
                if !point_valid_dist(split.split_vertex.pos(), &mut query_stack) {
                    continue;
                }

                // test segment midpoint
                let midpoint = seg_midpoint(split.updated_start, split.split_vertex);
                if !point_valid_dist(midpoint, &mut query_stack) {
                    continue;
                }

                // test intersection with original polyline
                if intersects_original_pline(
                    split.updated_start,
                    split.split_vertex,
                    &mut query_stack,
                ) {
                    continue;
                }

                // passed all tests, add the slice
                let mut slice = Polyline::new();
                slice.add_vertex(split.updated_start);
                slice.add_vertex(split.split_vertex);
                result.push(OpenPolylineSlice::new(start_index, slice));
            }
        }

        // build the slice between the last intersect in the intr_list and the next intersect found
        // check that the first point is valid
        let slice_start_point = *intr_list.last().unwrap();
        if !point_valid_dist(slice_start_point, &mut query_stack) {
            continue;
        }

        let split = seg_split_at_point(start_vertex, end_vertex, slice_start_point, pos_equal_eps);
        let mut slice = Polyline::new();
        slice.add_vertex(split.split_vertex);

        let mut index = next_index;
        let mut is_valid_pline = true;
        let mut loop_count = 0;
        let max_loop_count = raw_offset_polyline.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!("loop_count exceeded max_loop_count while creating slices from raw offset");
            }
            loop_count += 1;

            let current_vertex = raw_offset_polyline[index];
            // check that vertex point is valid
            if !point_valid_dist(current_vertex.pos(), &mut query_stack) {
                is_valid_pline = false;
                break;
            }

            // check that the segment does not intersect original polyline
            if intersects_original_pline(*slice.last().unwrap(), current_vertex, &mut query_stack) {
                is_valid_pline = false;
                break;
            }

            // add vertex
            slice.add_or_replace_vertex(current_vertex, pos_equal_eps);

            // check if segment that starts at current vertex just added to slice has an intersect
            if let Some(next_intr_list) = intersects_lookup.get(&index) {
                // there is an intersect, slice is done, check if final segment is valid

                // check intersect point is valid (which will be the end of the slice)
                let intersect_point = next_intr_list[0];
                if !point_valid_dist(intersect_point, &mut query_stack) {
                    is_valid_pline = false;
                    break;
                }

                let next_index = raw_offset_polyline.next_wrapping_index(index);
                let split = seg_split_at_point(
                    current_vertex,
                    raw_offset_polyline[next_index],
                    intersect_point,
                    pos_equal_eps,
                );

                let slice_end_vertex = PlineVertex::from_vector2(intersect_point, T::zero());
                // check midpoint is valid
                let midpoint = seg_midpoint(split.updated_start, slice_end_vertex);
                if !point_valid_dist(midpoint, &mut query_stack) {
                    is_valid_pline = false;
                    break;
                }

                // trim last added vertex and add final intersect point
                *slice.last_mut().unwrap() = split.updated_start;
                slice.add_or_replace_vertex(slice_end_vertex, pos_equal_eps);
                break;
            }
            // else there is not an intersect, increment index and continue
            if index == raw_offset_polyline.len() - 1 {
                if original_polyline.is_closed() {
                    // wrap index
                    index = 0;
                } else {
                    // open polyline, we're done
                    break;
                }
            } else {
                index += 1;
            }
        }

        if is_valid_pline && slice.len() > 1 {
            result.push(OpenPolylineSlice::new(start_index, slice));
        }
    }

    result
}

pub fn stitch_slices_together<T>(
    slices: &Vec<OpenPolylineSlice<T>>,
    is_closed: bool,
    orig_max_index: usize,
    options: &PlineOffsetOptions<T>,
) -> Vec<Polyline<T>>
where
    T: Real,
{
    let mut result = Vec::new();
    if slices.len() == 0 {
        return result;
    }

    let join_eps = options.slice_join_eps;
    let pos_equal_eps = options.pos_equal_eps;

    if slices.len() == 1 {
        let mut pline = slices[0].polyline.clone();

        if is_closed
            && pline[0]
                .pos()
                .fuzzy_eq_eps(pline.last().unwrap().pos(), join_eps)
        {
            pline.set_is_closed(true);
            pline.remove_last();
        }

        result.push(pline);
        return result;
    }

    let spatial_index = {
        let mut builder = StaticAABB2DIndexBuilder::new(slices.len());
        for slice in slices {
            let start_point = slice.polyline[0].pos();
            builder.add(
                start_point.x - join_eps,
                start_point.y - join_eps,
                start_point.x + join_eps,
                start_point.y + join_eps,
            );
        }
        builder.build().unwrap()
    };

    let mut visited_indexes = vec![false; slices.len()];
    let mut query_results = Vec::new();
    let mut query_stack = Vec::with_capacity(8);

    for i in 0..slices.len() {
        if visited_indexes[i] {
            continue;
        }

        visited_indexes[i] = true;

        let mut current_pline = Polyline::new();
        let mut current_index = i;
        let initial_start_point = slices[i].polyline[0].pos();
        let mut loop_count = 0;
        let max_loop_count = slices.len();
        loop {
            if loop_count > max_loop_count {
                // prevent infinite loop
                panic!("loop_count exceeded max_loop_count while stitching slices together");
            }
            loop_count += 1;

            let current_slice = &slices[current_index];
            current_pline.extend_vertexes(&current_slice.polyline);
            let current_loop_start_index = current_slice.intr_start_index;
            let current_end_point = current_slice.polyline.last().unwrap().pos();

            query_results.clear();
            let mut spatial_index_visitor = |i: usize| -> bool {
                if !visited_indexes[i] {
                    query_results.push(i);
                }
                true
            };
            spatial_index.visit_query_with_stack(
                current_end_point.x - join_eps,
                current_end_point.y - join_eps,
                current_end_point.x + join_eps,
                current_end_point.y + join_eps,
                &mut spatial_index_visitor,
                &mut query_stack,
            );

            let get_index_dist = |i: usize| -> usize {
                let slice = &slices[i];
                if current_loop_start_index <= slice.intr_start_index {
                    slice.intr_start_index - current_loop_start_index
                } else {
                    // forward wrapping distance (distance to end + distance to index)
                    orig_max_index - current_loop_start_index + slice.intr_start_index
                }
            };

            let end_connects_to_start = |i: usize| -> bool {
                let end_point = slices[i].polyline.last().unwrap().pos();
                end_point.fuzzy_eq_eps(initial_start_point, pos_equal_eps)
            };

            query_results.sort_unstable_by(|a, b| {
                // sort by index distance then by end of slice connecting to initial start
                // this ordering ensures overlapping slices are retained in stitching
                get_index_dist(*a)
                    .cmp(&get_index_dist(*b))
                    .then_with(|| end_connects_to_start(*a).cmp(&end_connects_to_start(*b)))
            });

            if query_results.is_empty() {
                // done stitching current polyline
                if current_pline.len() > 1 {
                    let current_pline_sp = current_pline[0].pos();
                    let current_pline_ep = current_pline.last().unwrap().pos();
                    if is_closed && current_pline_sp.fuzzy_eq_eps(current_pline_ep, pos_equal_eps) {
                        current_pline.remove_last();
                        current_pline.set_is_closed(true);
                    }

                    result.push(current_pline);
                }
                break;
            }

            // else continue stitching
            visited_indexes[query_results[0]] = true;
            current_pline.remove_last();
            current_index = query_results[0];
        }
    }

    result
}

pub fn parallel_offset<T>(
    polyline: &Polyline<T>,
    offset: T,
    spatial_index: Option<&StaticAABB2DIndex<T>>,
    options: Option<PlineOffsetOptions<T>>,
) -> Vec<Polyline<T>>
where
    T: Real,
{
    if polyline.len() < 2 {
        return Vec::new();
    }

    let opt = options.unwrap_or_default();

    let mut _constructed_index = None;
    let index = if let Some(x) = spatial_index {
        x
    } else {
        _constructed_index = Some(polyline.create_approx_spatial_index().unwrap());
        _constructed_index.as_ref().unwrap()
    };

    let raw_offset = create_raw_offset_polyline(polyline, offset, opt.pos_equal_eps);
    dbg!(raw_offset.len(), &raw_offset);
    if polyline.is_closed() && !opt.handle_self_intersects {
        let slices = slices_from_raw_offset(polyline, &raw_offset, index, offset, &opt);
        stitch_slices_together(&slices, true, raw_offset.len() - 1, &opt)
    } else {
        let dual_raw_offset = dbg!(create_raw_offset_polyline(
            polyline,
            -offset,
            opt.pos_equal_eps
        ));
        let slices = slices_from_dual_raw_offsets(
            polyline,
            &raw_offset,
            &dual_raw_offset,
            index,
            offset,
            &opt,
        );

        dbg!(&slices);
        dbg!(stitch_slices_together(
            &slices,
            polyline.is_closed(),
            raw_offset.len(),
            &opt
        ))
    }
}
