use std::collections::{BTreeMap, BTreeSet};

use static_aabb2d_index::{StaticAABB2DIndex, StaticAABB2DIndexBuilder};

use crate::{
    core::{
        math::{Vector2, dist_squared},
        traits::Real,
    },
    polyline::{
        BooleanOp, BooleanResultInfo, FindIntersectsOptions, PlineBasicIntersect, PlineCreation,
        PlineInversionView, PlineOffsetOptions, PlineOrientation, PlineSource, PlineSourceMut,
        PlineViewData, Polyline, internal::pline_offset::point_valid_for_offset, seg_closest_point,
        seg_midpoint,
    },
};

const SHAPE_BOOLEAN_POS_EQUAL_EPS: f64 = 1e-8;
const SHAPE_BOOLEAN_AREA_EPS: f64 = 1e-9;

/// Bibliographic notes for the polygon boolean model used by this module.
///
/// The shape boolean implementation is layered over the lower-level polyline clipping algorithm:
/// closed loops are regularized as filled 2D regions, and open polylines are clipped as zero-area
/// linework after the filled result is known. The signed-loop reconstruction follows the standard
/// polygon-overlay distinction between positive material and negative holes, as discussed in:
///
/// - F. Martinez, A. J. Rueda, and F. R. Feito, "A new algorithm for computing Boolean operations
///   on polygons", *Computers & Geosciences* 35(6), 1177-1185, 2009. DOI:
///   `10.1016/j.cageo.2008.08.009`.
/// - B. R. Vatti, "A generic solution to polygon clipping", *Communications of the ACM* 35(7),
///   56-63, 1992. DOI: `10.1145/129902.129906`.
/// - G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons", *ACM Transactions on
///   Graphics* 17(2), 71-83, 1998. DOI: `10.1145/274363.274364`.
/// - F. Liu, K. Hormann, and C. Loop, "Clipping simple polygons with degenerate intersections",
///   *Computers & Graphics: X* 2, 100007, 2019. DOI: `10.1016/j.cagx.2019.100007`.
///
/// These references are cited as design background. This crate also supports circular-arc bulges,
/// so the concrete clipping and reconstruction details are specific to cavalier_contours.
const _SHAPE_BOOLEAN_BIBLIOGRAPHY: () = ();

/// An offset polyline with spatial indexing and parent loop tracking.
///
/// This structure represents a single offset result from a parent polyline, containing
/// both the generated offset polyline with its spatial index and a reference to which
/// original input polyline it was derived from.
///
/// # Public Visibility
///
/// This struct is made public for visualization and testing purposes, allowing
/// intermediate offset results to be inspected during algorithm execution.
pub struct OffsetLoop<T: Real> {
    /// Index of the parent loop in the original input shape
    pub parent_loop_idx: usize,
    /// The offset polyline with its spatial index for fast intersection queries
    pub indexed_pline: IndexedPolyline<T>,
}

impl<T> Default for OffsetLoop<T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self {
            parent_loop_idx: Default::default(),
            indexed_pline: IndexedPolyline::new(Polyline::new()),
        }
    }
}

/// A polyline with an associated spatial index for efficient geometric queries.
///
/// This structure combines a polyline with a spatial index (AABB tree) that enables
/// fast intersection testing, nearest neighbor queries, and other spatial operations.
/// The spatial index is automatically built from the polyline's segment bounding boxes.
///
/// # Public Visibility
///
/// This struct is made public for visualization and testing purposes, allowing
/// access to both the polyline geometry and its spatial acceleration structure.
#[derive(Debug, Clone)]
pub struct IndexedPolyline<T: Real> {
    /// The polyline geometry
    pub polyline: Polyline<T>,
    /// Spatial index built from the polyline's segment bounding boxes
    pub spatial_index: StaticAABB2DIndex<T>,
}

impl<T> IndexedPolyline<T>
where
    T: Real,
{
    pub fn new(polyline: Polyline<T>) -> Self {
        let spatial_index = polyline.create_approx_aabb_index();
        Self {
            polyline,
            spatial_index,
        }
    }

    pub fn parallel_offset_for_shape(
        &self,
        offset: T,
        options: &ShapeOffsetOptions<T>,
    ) -> Vec<Polyline<T>> {
        let opts = PlineOffsetOptions {
            aabb_index: Some(&self.spatial_index),
            handle_self_intersects: false,
            pos_equal_eps: options.pos_equal_eps,
            slice_join_eps: options.slice_join_eps,
            offset_dist_eps: options.offset_dist_eps,
        };

        self.polyline.parallel_offset_opt(offset, &opts)
    }
}

/// Shape represented by positive area counter clockwise polylines, `ccw_plines` and negative/hole
/// area clockwise polylines, `cw_plines`.
#[derive(Debug, Clone)]
pub struct Shape<T: Real> {
    /// Positive/filled area counter clockwise polylines.
    pub ccw_plines: Vec<IndexedPolyline<T>>,
    /// Negative/hole area clockwise polylines.
    pub cw_plines: Vec<IndexedPolyline<T>>,
    /// Spatial index of all the polyline area bounding boxes, index positions correspond to in
    /// order all the counter clockwise polylines followed by all the clockwise polylines. E.g., if
    /// there is 1 `ccw_plines` and 2 `cw_plines` then index position 0 is the bounding box for the
    /// ccw pline and index positions 1 and 2 correspond to the first and second cw plines.
    pub plines_index: StaticAABB2DIndex<T>,
}

/// Struct to hold options parameters when performing shape offset.
#[derive(Debug, Clone)]
pub struct ShapeOffsetOptions<T> {
    /// Fuzzy comparison epsilon used for determining if two positions are equal.
    pub pos_equal_eps: T,
    /// Fuzzy comparison epsilon used when testing distance of slices to original polyline for
    /// validity.
    pub offset_dist_eps: T,
    /// Fuzzy comparison epsilon used for determining if two positions are equal when stitching
    /// polyline slices together.
    pub slice_join_eps: T,
}

/// A builder type for accumulating transformations on a `Shape`
/// without rebuilding indexes until the final `build` step.
pub struct ShapeTransformBuilder<T: Real> {
    shape: Shape<T>,
    scale_factor: T,
    rotate_angle: T,
    rotate_anchor: Option<(T, T)>,
    translate_x: T,
    translate_y: T,
    invert_direction: bool,
}

impl<T> ShapeOffsetOptions<T>
where
    T: Real,
{
    #[inline]
    pub fn new() -> Self {
        Self {
            pos_equal_eps: T::from(1e-5).unwrap(),
            offset_dist_eps: T::from(1e-4).unwrap(),
            slice_join_eps: T::from(1e-4).unwrap(),
        }
    }
}

impl<T> Default for ShapeOffsetOptions<T>
where
    T: Real,
{
    #[inline]
    fn default() -> Self {
        Self::new()
    }
}

impl<T> Shape<T>
where
    T: Real,
{
    fn is_empty_shape(&self) -> bool {
        self.ccw_plines.is_empty() && self.cw_plines.is_empty()
    }

    fn has_open_plines(&self) -> bool {
        self.ccw_plines
            .iter()
            .chain(self.cw_plines.iter())
            .any(|ip| !ip.polyline.is_closed())
    }

    fn without_open_plines(&self) -> Self {
        Self::from_plines(
            self.ccw_plines
                .iter()
                .chain(self.cw_plines.iter())
                .filter(|ip| ip.polyline.is_closed())
                .map(|ip| ip.polyline.clone()),
        )
    }

    fn open_plines(&self) -> impl Iterator<Item = &Polyline<T>> {
        self.ccw_plines
            .iter()
            .chain(self.cw_plines.iter())
            .filter(|ip| !ip.polyline.is_closed())
            .map(|ip| &ip.polyline)
    }

    fn point_on_closed_boundary(&self, point: Vector2<T>, eps: T) -> bool {
        self.ccw_plines
            .iter()
            .chain(self.cw_plines.iter())
            .filter(|ip| ip.polyline.is_closed())
            .any(|ip| {
                ip.polyline
                    .iter_segments()
                    .any(|(v1, v2)| seg_closest_point(v1, v2, point, eps).fuzzy_eq_eps(point, eps))
            })
    }

    fn contains_point_or_boundary(&self, point: Vector2<T>, eps: T) -> bool {
        self.ccw_plines
            .iter()
            .map(|ip| ip.polyline.winding_number(point))
            .chain(
                self.cw_plines
                    .iter()
                    .filter(|ip| ip.polyline.is_closed())
                    .map(|ip| ip.polyline.winding_number(point)),
            )
            .sum::<i32>()
            != 0
            || self.point_on_closed_boundary(point, eps)
    }

    fn segment_param(v1: Vector2<T>, v2: Vector2<T>, point: Vector2<T>) -> T {
        let dx = v2.x - v1.x;
        let dy = v2.y - v1.y;
        if dx.abs() >= dy.abs() {
            if dx.abs() > T::zero() {
                (point.x - v1.x) / dx
            } else {
                T::zero()
            }
        } else {
            (point.y - v1.y) / dy
        }
    }

    fn push_unique_param(params: &mut Vec<T>, value: T, eps: T) {
        if value < T::zero() - eps || value > T::one() + eps {
            return;
        }

        let clamped = if value < T::zero() {
            T::zero()
        } else if value > T::one() {
            T::one()
        } else {
            value
        };

        if !params.iter().any(|&t| t.fuzzy_eq_eps(clamped, eps)) {
            params.push(clamped);
        }
    }

    fn clip_open_line_segment_to_area(
        v1: Vector2<T>,
        v2: Vector2<T>,
        area_shape: &Self,
        keep_inside: bool,
        eps: T,
        result: &mut Vec<Polyline<T>>,
    ) {
        if v1.fuzzy_eq_eps(v2, eps) {
            return;
        }

        let mut params = vec![T::zero(), T::one()];
        let segment = {
            let mut pline = Polyline::new();
            pline.add(v1.x, v1.y, T::zero());
            pline.add(v2.x, v2.y, T::zero());
            pline
        };

        for boundary in area_shape
            .ccw_plines
            .iter()
            .chain(area_shape.cw_plines.iter())
            .filter(|ip| ip.polyline.is_closed())
            .map(|ip| &ip.polyline)
        {
            let intersects = segment.find_intersects(boundary);
            for intr in intersects.basic_intersects {
                Self::push_unique_param(&mut params, Self::segment_param(v1, v2, intr.point), eps);
            }
            for intr in intersects.overlapping_intersects {
                Self::push_unique_param(&mut params, Self::segment_param(v1, v2, intr.point1), eps);
                Self::push_unique_param(&mut params, Self::segment_param(v1, v2, intr.point2), eps);
            }
        }

        params.sort_by(|a, b| a.total_cmp(b));
        params.dedup_by(|a, b| a.fuzzy_eq_eps(*b, eps));

        let point_at = |t: T| Vector2::new(v1.x + (v2.x - v1.x) * t, v1.y + (v2.y - v1.y) * t);
        for pair in params.windows(2) {
            let start_t = pair[0];
            let end_t = pair[1];
            if (end_t - start_t).abs() <= eps {
                continue;
            }

            let mid_t = (start_t + end_t) / T::two();
            let midpoint = point_at(mid_t);
            if area_shape.contains_point_or_boundary(midpoint, eps) == keep_inside {
                let start = point_at(start_t);
                let end = point_at(end_t);
                let mut pline = Polyline::new();
                pline.add(start.x, start.y, T::zero());
                pline.add(end.x, end.y, T::zero());
                result.push(pline);
            }
        }
    }

    fn clip_open_plines_to_area<'a>(
        plines: impl Iterator<Item = &'a Polyline<T>>,
        area_shape: &Self,
        keep_inside: bool,
    ) -> Vec<Polyline<T>> {
        let eps = T::from(SHAPE_BOOLEAN_POS_EQUAL_EPS).unwrap();
        let mut result = Vec::new();

        for pline in plines {
            for (v1, v2) in pline.iter_segments() {
                if v1.bulge.fuzzy_eq_eps(T::zero(), eps) {
                    Self::clip_open_line_segment_to_area(
                        v1.pos(),
                        v2.pos(),
                        area_shape,
                        keep_inside,
                        eps,
                        &mut result,
                    );
                } else if area_shape.contains_point_or_boundary(seg_midpoint(v1, v2), eps)
                    == keep_inside
                {
                    let mut arc = Polyline::new();
                    arc.add(v1.x, v1.y, v1.bulge);
                    arc.add(v2.x, v2.y, T::zero());
                    result.push(arc);
                }
            }
        }

        result
    }

    fn boolean_with_open_plines(&self, other: &Self, op: BooleanOp) -> Self {
        let self_area = self.without_open_plines();
        let other_area = other.without_open_plines();
        let area_result = self_area.boolean(&other_area, op);
        let mut candidate_lines = Vec::new();

        // Open polylines carry no area, so they cannot affect the filled boolean result. Treat
        // them as linework that is visible only where the corresponding set operation would keep
        // a zero-area feature. A final clip against `area_result` removes linework already covered
        // by filled material, matching the UI expectation that filled regions absorb internal
        // construction lines.
        match op {
            BooleanOp::Or | BooleanOp::Xor => {
                candidate_lines.extend(Self::clip_open_plines_to_area(
                    self.open_plines(),
                    &other_area,
                    false,
                ));
                candidate_lines.extend(Self::clip_open_plines_to_area(
                    other.open_plines(),
                    &self_area,
                    false,
                ));
            }
            BooleanOp::And => {
                candidate_lines.extend(Self::clip_open_plines_to_area(
                    self.open_plines(),
                    &other_area,
                    true,
                ));
                candidate_lines.extend(Self::clip_open_plines_to_area(
                    other.open_plines(),
                    &self_area,
                    true,
                ));
            }
            BooleanOp::Not => {
                candidate_lines.extend(Self::clip_open_plines_to_area(
                    self.open_plines(),
                    &other_area,
                    false,
                ));
            }
        }

        let visible_lines =
            Self::clip_open_plines_to_area(candidate_lines.iter(), &area_result, false);
        let mut result_plines = area_result
            .ccw_plines
            .into_iter()
            .chain(area_result.cw_plines)
            .map(|ip| ip.polyline)
            .collect::<Vec<_>>();
        result_plines.extend(visible_lines);

        Self::from_plines(result_plines)
    }

    fn same_loop_geometry(a: &Polyline<T>, b: &Polyline<T>) -> bool {
        if a.is_closed() != b.is_closed()
            || a.vertex_count() != b.vertex_count()
            || a.orientation() != b.orientation()
        {
            return false;
        }

        let epsilon = T::from(SHAPE_BOOLEAN_POS_EQUAL_EPS).unwrap();

        // Closed polylines can represent the same loop with any vertex as the start. Keep the
        // comparison geometric for identity fast paths while still requiring matching orientation
        // through the signed loop bins that call this helper.
        let vertexes_equal = |a_idx: usize, b_idx: usize| {
            let av = a.at(a_idx);
            let bv = b.at(b_idx);
            av.x.fuzzy_eq_eps(bv.x, epsilon)
                && av.y.fuzzy_eq_eps(bv.y, epsilon)
                && av.bulge.fuzzy_eq_eps(bv.bulge, epsilon)
        };

        if a.is_closed() {
            return (0..a.vertex_count()).any(|start_idx| {
                (0..a.vertex_count()).all(|i| vertexes_equal(i, (start_idx + i) % a.vertex_count()))
            });
        }

        (0..a.vertex_count()).all(|i| {
            let av = a.at(i);
            let bv = b.at(i);
            av.x.fuzzy_eq_eps(bv.x, epsilon)
                && av.y.fuzzy_eq_eps(bv.y, epsilon)
                && av.bulge.fuzzy_eq_eps(bv.bulge, epsilon)
        })
    }

    /// Compare signed-loop bins geometrically instead of by storage order.
    ///
    /// Shape boolean has identity fast paths for equal operands. Loop order and closed-loop start
    /// vertex are not semantic, so this comparison deliberately accepts reordered and rotated
    /// loops while still keeping CCW material and CW holes in separate bins.
    fn same_loop_set(a: &[IndexedPolyline<T>], b: &[IndexedPolyline<T>]) -> bool {
        if a.len() != b.len() {
            return false;
        }

        let mut b_matched = vec![false; b.len()];
        'next_a: for a_loop in a {
            for (b_idx, b_loop) in b.iter().enumerate() {
                if !b_matched[b_idx] && Self::same_loop_geometry(&a_loop.polyline, &b_loop.polyline)
                {
                    b_matched[b_idx] = true;
                    continue 'next_a;
                }
            }

            return false;
        }

        true
    }

    /// Return true when both shapes contain the same CCW material loops and CW hole loops.
    fn same_loop_bins(&self, other: &Self) -> bool {
        Self::same_loop_set(&self.ccw_plines, &other.ccw_plines)
            && Self::same_loop_set(&self.cw_plines, &other.cw_plines)
    }

    /// Area tolerance used only while assembling shape boolean results.
    ///
    /// Lower-level polyline booleans can emit near-zero slivers around tangencies and shared
    /// boundaries. Keeping this threshold centralized prevents the four signed-loop pairings from
    /// drifting apart as the shape-level assembly evolves.
    #[inline]
    fn shape_boolean_area_epsilon() -> T {
        T::from(SHAPE_BOOLEAN_AREA_EPS).unwrap()
    }

    /// Normalize a result loop into the positive-material orientation expected by `ccw_plines`.
    fn normalize_ccw(pline: Polyline<T>) -> Polyline<T> {
        if pline.orientation() == PlineOrientation::Clockwise {
            Polyline::create_from(&PlineInversionView::new(&pline))
        } else {
            pline
        }
    }

    /// Normalize a result loop into the subtractive-hole orientation expected by `cw_plines`.
    fn normalize_cw(pline: Polyline<T>) -> Polyline<T> {
        if pline.orientation() == PlineOrientation::CounterClockwise {
            Polyline::create_from(&PlineInversionView::new(&pline))
        } else {
            pline
        }
    }

    /// Build a shape from already classified signed loops.
    ///
    /// All shape boolean exits go through this helper so orientation normalization, tiny-area
    /// filtering, child indexes, and the top-level shape index stay consistent. Reusing this path
    /// also keeps docs.rs output stable by avoiding duplicate private assembly variants.
    fn build_from_signed_plines(ccw_plines: Vec<Polyline<T>>, cw_plines: Vec<Polyline<T>>) -> Self {
        let area_eps = Self::shape_boolean_area_epsilon();
        let final_ccw_result = ccw_plines
            .into_iter()
            .map(Self::normalize_ccw)
            .filter(|pline| pline.area().abs() > area_eps)
            .map(IndexedPolyline::new)
            .collect::<Vec<_>>();
        let final_cw_result = cw_plines
            .into_iter()
            .map(Self::normalize_cw)
            .filter(|pline| pline.area().abs() > area_eps)
            .map(IndexedPolyline::new)
            .collect::<Vec<_>>();

        let mut builder =
            StaticAABB2DIndexBuilder::new(final_ccw_result.len() + final_cw_result.len());
        for ip in &final_ccw_result {
            if let Some(bds) = ip.spatial_index.bounds() {
                builder.add(bds.min_x, bds.min_y, bds.max_x, bds.max_y);
            }
        }
        for ip in &final_cw_result {
            if let Some(bds) = ip.spatial_index.bounds() {
                builder.add(bds.min_x, bds.min_y, bds.max_x, bds.max_y);
            }
        }
        let plines_index = builder.build().unwrap();

        Self {
            ccw_plines: final_ccw_result,
            cw_plines: final_cw_result,
            plines_index,
        }
    }

    /// Merge positive material loops using the existing polyline union implementation.
    ///
    /// Shape booleans may accumulate many pairwise pieces for one final island. This helper keeps
    /// that consolidation local while preserving the lower-level regularization rules for
    /// edge-touching loops.
    fn merge_ccw_plines(plines: Vec<Polyline<T>>) -> Vec<Polyline<T>> {
        let area_eps = Self::shape_boolean_area_epsilon();
        let mut merged: Vec<Polyline<T>> = Vec::new();

        'next_pline: for pline in plines {
            let mut pending = Self::normalize_ccw(pline);

            let mut i = 0;
            while i < merged.len() {
                let result = merged[i].boolean(&pending, BooleanOp::Or);
                if matches!(result.result_info, BooleanResultInfo::Disjoint) {
                    i += 1;
                    continue;
                }

                if !matches!(
                    result.result_info,
                    BooleanResultInfo::Intersected | BooleanResultInfo::Overlapping
                ) {
                    i += 1;
                    continue;
                }

                let mut pos_plines = result
                    .pos_plines
                    .into_iter()
                    .map(|rp| Self::normalize_ccw(rp.pline))
                    .collect::<Vec<_>>();

                if pos_plines.len() == 1 {
                    merged.remove(i);
                    pending = pos_plines.remove(0);
                    continue;
                }

                if pos_plines.is_empty() {
                    merged.remove(i);
                    continue 'next_pline;
                }

                // The lower polyline boolean can intentionally leave edge-touching regularized
                // regions as separate loops. Keep that behavior at shape level instead of
                // inventing a stronger merge rule here.
                i += 1;
            }

            if pending.area().abs() > area_eps {
                merged.push(pending);
            }
        }

        merged
    }

    fn subtract_positive_plines(
        mut pieces: Vec<Polyline<T>>,
        cutters: &[IndexedPolyline<T>],
    ) -> Vec<Polyline<T>> {
        let area_eps = Self::shape_boolean_area_epsilon();

        for cutter in cutters {
            let mut next_pieces = Vec::new();
            for piece in pieces {
                let result = piece.boolean(&cutter.polyline, BooleanOp::Not);
                next_pieces.extend(
                    result
                        .pos_plines
                        .into_iter()
                        .map(|rp| Self::normalize_ccw(rp.pline))
                        .filter(|pline| pline.area().abs() > area_eps),
                );
            }

            pieces = next_pieces;
            if pieces.is_empty() {
                break;
            }
        }

        pieces
    }

    fn boolean_or(&self, other: &Self) -> Self {
        let mut hole_regions = Vec::new();
        let mut final_ccw = if self.ccw_plines.len() == 1 && other.ccw_plines.len() == 1 {
            // For one shell per operand, the lower-level union can report holes created by arc
            // overlap directly as negative loops. Preserve them separately from user-authored CW
            // holes so they can all be clipped back to the final material at the end.
            let shell_union = self.ccw_plines[0]
                .polyline
                .boolean(&other.ccw_plines[0].polyline, BooleanOp::Or);
            hole_regions.extend(
                shell_union
                    .neg_plines
                    .into_iter()
                    .map(|rp| Self::normalize_ccw(rp.pline)),
            );
            shell_union
                .pos_plines
                .into_iter()
                .map(|rp| Self::normalize_ccw(rp.pline))
                .collect()
        } else {
            Self::merge_ccw_plines(
                self.ccw_plines
                    .iter()
                    .chain(other.ccw_plines.iter())
                    .map(|ip| Self::normalize_ccw(ip.polyline.clone()))
                    .collect(),
            )
        };

        if self.ccw_plines.len() == 1 && other.ccw_plines.len() == 1 {
            if self.cw_plines.iter().any(|hole| {
                Self::pline_fully_contains_pline_area(
                    &Self::normalize_ccw(hole.polyline.clone()),
                    &other.ccw_plines[0].polyline,
                )
            }) {
                final_ccw.push(Self::normalize_ccw(other.ccw_plines[0].polyline.clone()));
            }

            if other.cw_plines.iter().any(|hole| {
                Self::pline_fully_contains_pline_area(
                    &Self::normalize_ccw(hole.polyline.clone()),
                    &self.ccw_plines[0].polyline,
                )
            }) {
                final_ccw.push(Self::normalize_ccw(self.ccw_plines[0].polyline.clone()));
            }
        }

        for hole in &self.cw_plines {
            let hole_positive = Self::normalize_ccw(hole.polyline.clone());
            let nested_self_material = self
                .ccw_plines
                .iter()
                .filter(|material| {
                    Self::pline_fully_contains_pline_area(&hole_positive, &material.polyline)
                })
                .cloned()
                .collect::<Vec<_>>();
            let hole_pieces =
                Self::subtract_positive_plines(vec![hole_positive], &nested_self_material);
            hole_regions.extend(Self::subtract_positive_plines(
                hole_pieces,
                &other.ccw_plines,
            ));
        }
        for hole in &other.cw_plines {
            let hole_positive = Self::normalize_ccw(hole.polyline.clone());
            let nested_other_material = other
                .ccw_plines
                .iter()
                .filter(|material| {
                    Self::pline_fully_contains_pline_area(&hole_positive, &material.polyline)
                })
                .cloned()
                .collect::<Vec<_>>();
            let hole_pieces =
                Self::subtract_positive_plines(vec![hole_positive], &nested_other_material);
            hole_regions.extend(Self::subtract_positive_plines(
                hole_pieces,
                &self.ccw_plines,
            ));
        }
        for self_hole in &self.cw_plines {
            let self_hole_positive = Self::normalize_ccw(self_hole.polyline.clone());
            for other_hole in &other.cw_plines {
                let other_hole_positive = Self::normalize_ccw(other_hole.polyline.clone());
                let result = self_hole_positive.boolean(&other_hole_positive, BooleanOp::And);
                // In a union, empty space survives only where both operands are empty. Intersect
                // positive views of the holes to recover exactly that shared empty region.
                hole_regions.extend(
                    result
                        .pos_plines
                        .into_iter()
                        .map(|rp| Self::normalize_ccw(rp.pline)),
                );
            }
        }

        let final_ccw_shape = Self::build_from_signed_plines(final_ccw.clone(), Vec::new());
        let hole_regions = Self::merge_ccw_plines(hole_regions)
            .into_iter()
            .flat_map(|hole| {
                let nested_material = final_ccw_shape
                    .ccw_plines
                    .iter()
                    .filter(|material| {
                        Self::pline_fully_contains_pline_area(&hole, &material.polyline)
                    })
                    .cloned()
                    .collect::<Vec<_>>();
                Self::subtract_positive_plines(vec![hole], &nested_material)
            })
            .collect::<Vec<_>>();
        let final_cw = Self::merge_ccw_plines(hole_regions)
            .into_iter()
            .filter(|pline| {
                // Hole computations can produce regions outside the unioned shells when an input
                // hole crosses a shell edge. Drop anything with no sampled interior inside final
                // material before returning it as a subtractive CW loop.
                Self::pline_has_area_where(pline, |point| {
                    final_ccw_shape.contains_point_or_boundary(
                        point,
                        T::from(SHAPE_BOOLEAN_POS_EQUAL_EPS).unwrap(),
                    )
                })
            })
            .map(Self::normalize_cw)
            .collect();

        Self::build_from_signed_plines(final_ccw, final_cw)
    }

    fn filter_holes_to_material(
        ccw_plines: &[Polyline<T>],
        cw_plines: Vec<Polyline<T>>,
    ) -> Vec<Polyline<T>> {
        let mut clipped_holes = Vec::new();

        for hole in cw_plines {
            let positive_hole = Self::normalize_ccw(hole);
            for material in ccw_plines {
                // A CW loop is meaningful only inside positive material. Clip every candidate
                // hole to each material island before merging; this prevents detached negative
                // loops from subtracting area outside any shell.
                let result = positive_hole.boolean(material, BooleanOp::And);
                clipped_holes.extend(
                    result
                        .pos_plines
                        .into_iter()
                        .map(|rp| Self::normalize_ccw(rp.pline)),
                );
            }
        }

        Self::merge_ccw_plines(clipped_holes)
            .into_iter()
            .map(Self::normalize_cw)
            .collect()
    }

    fn subtract_holes_from_signed_plines(
        ccw_plines: Vec<Polyline<T>>,
        cw_plines: Vec<Polyline<T>>,
        holes: &[IndexedPolyline<T>],
    ) -> (Vec<Polyline<T>>, Vec<Polyline<T>>) {
        let area_eps = Self::shape_boolean_area_epsilon();
        let mut ccw_result = ccw_plines;
        let mut cw_result = cw_plines;

        for hole in holes {
            let hole_positive = Self::normalize_ccw(hole.polyline.clone());
            let mut next_ccw = Vec::new();

            for pline in ccw_result {
                // Holes are stored clockwise in Shape, but the lower-level boolean expects
                // positive-area contours. Invert the hole, subtract it from material pieces, and
                // retain any negative loops that the clipping step introduces.
                let result = pline.boolean(&hole_positive, BooleanOp::Not);
                next_ccw.extend(
                    result
                        .pos_plines
                        .into_iter()
                        .map(|rp| Self::normalize_ccw(rp.pline))
                        .filter(|pline| pline.area().abs() > area_eps),
                );
                cw_result.extend(
                    result
                        .neg_plines
                        .into_iter()
                        .map(|rp| Self::normalize_cw(rp.pline))
                        .filter(|pline| pline.area().abs() > area_eps),
                );
            }

            ccw_result = next_ccw;
        }

        (ccw_result, cw_result)
    }

    fn boolean_and_single_shells(&self, other: &Self) -> Self {
        let shell_intersection = self.ccw_plines[0]
            .polyline
            .boolean(&other.ccw_plines[0].polyline, BooleanOp::And);
        let ccw_plines = shell_intersection
            .pos_plines
            .into_iter()
            .map(|rp| Self::normalize_ccw(rp.pline))
            .collect::<Vec<_>>();
        let cw_plines = shell_intersection
            .neg_plines
            .into_iter()
            .map(|rp| Self::normalize_cw(rp.pline))
            .collect::<Vec<_>>();

        let (ccw_plines, cw_plines) =
            Self::subtract_holes_from_signed_plines(ccw_plines, cw_plines, &self.cw_plines);
        let (ccw_plines, cw_plines) =
            Self::subtract_holes_from_signed_plines(ccw_plines, cw_plines, &other.cw_plines);

        let ccw_plines = Self::merge_ccw_plines(ccw_plines);
        let cw_plines = Self::filter_holes_to_material(&ccw_plines, cw_plines);

        Self::build_from_signed_plines(ccw_plines, cw_plines)
    }

    fn boolean_not_single_shells(&self, other: &Self) -> Self {
        let effective_other_shell = Self::subtract_positive_plines(
            vec![other.ccw_plines[0].polyline.clone()],
            &self.cw_plines,
        );
        let mut ccw_plines = vec![self.ccw_plines[0].polyline.clone()];
        let mut cw_plines = Vec::new();

        for cutter in effective_other_shell {
            let mut next_ccw = Vec::new();
            for pline in ccw_plines {
                let result = pline.boolean(&cutter, BooleanOp::Not);
                next_ccw.extend(
                    result
                        .pos_plines
                        .into_iter()
                        .map(|rp| Self::normalize_ccw(rp.pline)),
                );
                cw_plines.extend(
                    result
                        .neg_plines
                        .into_iter()
                        .map(|rp| Self::normalize_cw(rp.pline)),
                );
            }
            ccw_plines = next_ccw;
        }

        for other_hole in &other.cw_plines {
            let other_hole_positive = Self::normalize_ccw(other_hole.polyline.clone());
            let result = self.ccw_plines[0]
                .polyline
                .boolean(&other_hole_positive, BooleanOp::And);
            ccw_plines.extend(
                result
                    .pos_plines
                    .into_iter()
                    .map(|rp| Self::normalize_ccw(rp.pline)),
            );
        }

        let (ccw_plines, cw_plines) =
            Self::subtract_holes_from_signed_plines(ccw_plines, cw_plines, &self.cw_plines);

        let ccw_plines = Self::merge_ccw_plines(ccw_plines);
        let cw_plines = Self::filter_holes_to_material(&ccw_plines, cw_plines);

        Self::build_from_signed_plines(ccw_plines, cw_plines)
    }

    fn boolean_xor_single_shells(&self, other: &Self) -> Self {
        let other_shell_inside_self_hole = self.cw_plines.iter().any(|hole| {
            Self::pline_fully_contains_pline_area(
                &Self::normalize_ccw(hole.polyline.clone()),
                &other.ccw_plines[0].polyline,
            )
        });
        let self_shell_inside_other_hole = other.cw_plines.iter().any(|hole| {
            Self::pline_fully_contains_pline_area(
                &Self::normalize_ccw(hole.polyline.clone()),
                &self.ccw_plines[0].polyline,
            )
        });

        if other_shell_inside_self_hole || self_shell_inside_other_hole {
            return self.boolean_or(other);
        }

        let left_shell_difference = self.ccw_plines[0]
            .polyline
            .boolean(&other.ccw_plines[0].polyline, BooleanOp::Not);
        let right_shell_difference = other.ccw_plines[0]
            .polyline
            .boolean(&self.ccw_plines[0].polyline, BooleanOp::Not);

        let mut ccw_plines = left_shell_difference
            .pos_plines
            .into_iter()
            .chain(right_shell_difference.pos_plines)
            .map(|rp| Self::normalize_ccw(rp.pline))
            .collect::<Vec<_>>();
        let mut cw_plines = left_shell_difference
            .neg_plines
            .into_iter()
            .chain(right_shell_difference.neg_plines)
            .map(|rp| Self::normalize_cw(rp.pline))
            .collect::<Vec<_>>();

        let (left_ccw, left_cw) =
            Self::subtract_holes_from_signed_plines(ccw_plines, cw_plines, &self.cw_plines);
        let (mut outside_ccw, mut outside_cw) =
            Self::subtract_holes_from_signed_plines(left_ccw, left_cw, &other.cw_plines);

        for self_hole in &self.cw_plines {
            let self_hole_positive = Self::normalize_ccw(self_hole.polyline.clone());
            let result = self_hole_positive.boolean(&other.ccw_plines[0].polyline, BooleanOp::And);
            let hole_overlap = result
                .pos_plines
                .into_iter()
                .map(|rp| Self::normalize_ccw(rp.pline))
                .collect::<Vec<_>>();
            let (hole_overlap, overlap_holes) =
                Self::subtract_holes_from_signed_plines(hole_overlap, Vec::new(), &other.cw_plines);
            outside_ccw.extend(hole_overlap);
            outside_cw.extend(overlap_holes);
        }

        for other_hole in &other.cw_plines {
            let other_hole_positive = Self::normalize_ccw(other_hole.polyline.clone());
            let result = other_hole_positive.boolean(&self.ccw_plines[0].polyline, BooleanOp::And);
            let hole_overlap = result
                .pos_plines
                .into_iter()
                .map(|rp| Self::normalize_ccw(rp.pline))
                .collect::<Vec<_>>();
            let (hole_overlap, overlap_holes) =
                Self::subtract_holes_from_signed_plines(hole_overlap, Vec::new(), &self.cw_plines);
            outside_ccw.extend(hole_overlap);
            outside_cw.extend(overlap_holes);
        }

        ccw_plines = outside_ccw;
        cw_plines = Self::filter_holes_to_material(&ccw_plines, outside_cw);

        Self::build_from_signed_plines(ccw_plines, cw_plines)
    }

    fn pline_contains_probe(container: &Polyline<T>, candidate: &Polyline<T>) -> bool {
        let extents = match candidate.extents() {
            Some(extents) => extents,
            None => return false,
        };
        let center = Vector2::new(
            (extents.min_x + extents.max_x) / T::two(),
            (extents.min_y + extents.max_y) / T::two(),
        );

        container.winding_number(center) != 0
            || candidate
                .iter_vertexes()
                .any(|v| container.winding_number(v.pos()) != 0)
            || candidate
                .iter_segments()
                .any(|(v1, v2)| container.winding_number(seg_midpoint(v1, v2)) != 0)
    }

    fn pline_fully_contains_pline_area(container: &Polyline<T>, candidate: &Polyline<T>) -> bool {
        let extents = match candidate.extents() {
            Some(extents) => extents,
            None => return false,
        };
        let center = Vector2::new(
            (extents.min_x + extents.max_x) / T::two(),
            (extents.min_y + extents.max_y) / T::two(),
        );

        container.winding_number(center) != 0
            && candidate
                .iter_vertexes()
                .all(|v| container.winding_number(v.pos()) != 0)
            && candidate
                .iter_segments()
                .all(|(v1, v2)| container.winding_number(seg_midpoint(v1, v2)) != 0)
    }

    fn pline_has_area_where(
        pline: &Polyline<T>,
        mut predicate: impl FnMut(Vector2<T>) -> bool,
    ) -> bool {
        let Some(extents) = pline.extents() else {
            return false;
        };

        let fractions = [
            T::from(0.17).unwrap(),
            T::from(0.31).unwrap(),
            T::from(0.47).unwrap(),
            T::from(0.63).unwrap(),
            T::from(0.79).unwrap(),
        ];
        let width = extents.max_x - extents.min_x;
        let height = extents.max_y - extents.min_y;

        for fx in fractions {
            for fy in fractions {
                let point = Vector2::new(extents.min_x + width * fx, extents.min_y + height * fy);
                if pline.winding_number(point) != 0 && predicate(point) {
                    return true;
                }
            }
        }

        false
    }

    fn canonicalize_standalone_cw_loops(&self) -> Option<Self> {
        let mut ccw_plines = self
            .ccw_plines
            .iter()
            .map(|ip| ip.polyline.clone())
            .collect::<Vec<_>>();
        let mut cw_plines = Vec::new();
        let mut changed = false;

        for cw in &self.cw_plines {
            let is_hole = self
                .ccw_plines
                .iter()
                .any(|ccw| Self::pline_contains_probe(&ccw.polyline, &cw.polyline));

            if is_hole {
                cw_plines.push(cw.polyline.clone());
            } else {
                ccw_plines.push(Self::normalize_ccw(cw.polyline.clone()));
                changed = true;
            }
        }

        changed.then(|| Self::build_from_signed_plines(ccw_plines, cw_plines))
    }

    /// Visit only loop indexes whose shape-level bounds overlap the query bounds.
    ///
    /// The top-level index stores CCW loops first and CW loops second. This helper hides that
    /// layout from the boolean pair collectors while preserving the same signed-bin semantics as a
    /// direct nested loop scan.
    #[inline]
    fn visit_loop_candidates(
        shape: &Shape<T>,
        bounds: &static_aabb2d_index::AABB<T>,
        want_ccw: bool,
        query_stack: &mut Vec<usize>,
        visitor: &mut impl FnMut(usize),
    ) {
        let ccw_len = shape.ccw_plines.len();
        // Shape indexes are built with CCW loops first, followed by CW loops, so query hits
        // can be filtered by global index and mapped back into the requested signed bin.
        let mut index_visitor = |idx| {
            if want_ccw {
                if idx < ccw_len {
                    visitor(idx);
                }
            } else if idx >= ccw_len {
                visitor(idx - ccw_len);
            }
        };
        shape.plines_index.visit_query_with_stack(
            bounds.min_x,
            bounds.min_y,
            bounds.max_x,
            bounds.max_y,
            &mut index_visitor,
            query_stack,
        );
    }

    /// Construct a `Shape` from polylines, sorting CCW loops into filled material and CW loops
    /// into holes.
    ///
    /// Empty polylines are ignored. If the loop bins are already known, construct `Shape`
    /// directly to avoid reclassifying by signed area.
    pub fn from_plines<I>(plines: I) -> Self
    where
        I: IntoIterator<Item = Polyline<T>>,
    {
        let mut ccw_plines = Vec::new();
        let mut cw_plines = Vec::new();
        // skip empty polylines
        for pl in plines.into_iter().filter(|p| p.vertex_count() > 1) {
            if pl.orientation() == PlineOrientation::CounterClockwise {
                ccw_plines.push(IndexedPolyline::new(pl));
            } else {
                cw_plines.push(IndexedPolyline::new(pl));
            }
        }

        let plines_index = {
            let mut b = StaticAABB2DIndexBuilder::new(ccw_plines.len() + cw_plines.len());

            let mut add_all_bounds = |plines: &[IndexedPolyline<T>]| {
                for pline in plines.iter() {
                    let bounds = pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_plines);
            add_all_bounds(&cw_plines);

            b.build().unwrap()
        };

        Self {
            ccw_plines,
            cw_plines,
            plines_index,
        }
    }

    /// Return an empty shape (0 polylines).
    #[inline]
    pub fn empty() -> Self {
        Self {
            ccw_plines: Vec::new(),
            cw_plines: Vec::new(),
            plines_index: StaticAABB2DIndexBuilder::new(0).build().unwrap(),
        }
    }

    pub fn parallel_offset(&self, offset: T, options: ShapeOffsetOptions<T>) -> Self {
        let (ccw_offset_loops, cw_offset_loops, offset_loops_index) =
            self.create_offset_loops_with_index(offset, &options);

        if ccw_offset_loops.is_empty() && cw_offset_loops.is_empty() {
            return Self::empty();
        }

        let slice_point_sets = self.find_intersects_between_offset_loops(
            &ccw_offset_loops,
            &cw_offset_loops,
            &offset_loops_index,
            options.pos_equal_eps,
        );

        let slices_data = self.create_valid_slices_from_intersects(
            &ccw_offset_loops,
            &cw_offset_loops,
            &slice_point_sets,
            offset,
            &options,
        );

        self.stitch_slices_together(
            slices_data,
            &ccw_offset_loops,
            &cw_offset_loops,
            options.pos_equal_eps,
            options.slice_join_eps,
        )
    }

    /// **Step 1** of the multipolyline offset algorithm: Creates offset loops with spatial index.
    ///
    /// This method generates offset polylines for each input polyline in the shape and creates
    /// a spatial index for efficient intersection queries. The offset loops are separated into
    /// counter-clockwise (positive area) and clockwise (negative area) collections based on
    /// their orientation after offsetting.
    ///
    /// # Returns
    ///
    /// A tuple containing:
    /// - `Vec<OffsetLoop<T>>` - Counter-clockwise offset loops
    /// - `Vec<OffsetLoop<T>>` - Clockwise offset loops
    /// - `StaticAABB2DIndex<T>` - Spatial index of all offset loop bounding boxes
    ///
    /// # Public Visibility
    ///
    /// This method is made public for visualization and testing purposes, allowing intermediate
    /// results to be inspected during the offset algorithm execution.
    pub fn create_offset_loops_with_index(
        &self,
        offset: T,
        options: &ShapeOffsetOptions<T>,
    ) -> (Vec<OffsetLoop<T>>, Vec<OffsetLoop<T>>, StaticAABB2DIndex<T>) {
        let mut ccw_offset_loops = Vec::new();
        let mut cw_offset_loops = Vec::new();
        let mut parent_idx = 0;

        for pline in self.ccw_plines.iter() {
            for offset_pline in pline.parallel_offset_for_shape(offset, options) {
                let area = offset_pline.area();
                // check if orientation inverted (due to collapse of very narrow or small input)
                // skip if inversion happened (ccw became cw while offsetting inward)
                if offset > T::zero() && area < T::zero() {
                    continue;
                }

                let offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline),
                };

                if area < T::zero() {
                    cw_offset_loops.push(offset_loop);
                } else {
                    ccw_offset_loops.push(offset_loop);
                }
            }
            parent_idx += 1;
        }

        for pline in self.cw_plines.iter() {
            for offset_pline in pline.parallel_offset_for_shape(offset, options) {
                let area = offset_pline.area();
                // check if orientation inverted (due to collapse of very narrow or small input)
                // skip if inversion happened (cw became ccw while offsetting inward)
                if offset < T::zero() && area > T::zero() {
                    continue;
                }

                let offset_loop = OffsetLoop {
                    parent_loop_idx: parent_idx,
                    indexed_pline: IndexedPolyline::new(offset_pline),
                };

                if area < T::zero() {
                    cw_offset_loops.push(offset_loop);
                } else {
                    ccw_offset_loops.push(offset_loop);
                }
            }
            parent_idx += 1;
        }

        let offset_loops_index = {
            let mut b =
                StaticAABB2DIndexBuilder::new(ccw_offset_loops.len() + cw_offset_loops.len());

            let mut add_all_bounds = |loops: &[OffsetLoop<T>]| {
                for l in loops.iter() {
                    let bounds = l
                        .indexed_pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_offset_loops);
            add_all_bounds(&cw_offset_loops);

            b.build()
                .expect("failed to build spatial index of offset loop bounds")
        };

        (ccw_offset_loops, cw_offset_loops, offset_loops_index)
    }

    /// **Step 2** of the multipolyline offset algorithm: Finds intersections between offset loops.
    ///
    /// This method uses spatial indexing to efficiently find all intersection points between
    /// the offset polylines generated in Step 1. It performs pairwise intersection tests only
    /// on polylines whose bounding boxes overlap, avoiding expensive computations on non-intersecting
    /// pairs. Both basic intersections and overlapping segments are detected and converted into
    /// slice points for further processing.
    ///
    /// # Arguments
    ///
    /// * `ccw_offset_loops` - Counter-clockwise offset loops from Step 1
    /// * `cw_offset_loops` - Clockwise offset loops from Step 1
    /// * `offset_loops_index` - Spatial index of offset loop bounding boxes from Step 1
    /// * `pos_equal_eps` - Epsilon for position equality comparisons
    ///
    /// # Returns
    ///
    /// A vector of `SlicePointSet<T>` containing intersection data between pairs of offset loops.
    /// Each set includes the loop indices and all intersection points between those loops.
    ///
    /// # Public Visibility
    ///
    /// This method is made public for visualization and testing purposes, allowing intersection
    /// points to be displayed and the intersection detection logic to be independently tested.
    pub fn find_intersects_between_offset_loops(
        &self,
        ccw_offset_loops: &[OffsetLoop<T>],
        cw_offset_loops: &[OffsetLoop<T>],
        offset_loops_index: &StaticAABB2DIndex<T>,
        pos_equal_eps: T,
    ) -> Vec<SlicePointSet<T>> {
        let offset_loop_count = ccw_offset_loops.len() + cw_offset_loops.len();
        let mut slice_point_sets = Vec::new();
        let mut visited_loop_pairs = BTreeSet::<(usize, usize)>::new();
        let mut query_stack = Vec::new();

        for i in 0..offset_loop_count {
            let loop1 = Self::get_loop(i, ccw_offset_loops, cw_offset_loops);
            let spatial_idx1 = &loop1.indexed_pline.spatial_index;
            let bounds = spatial_idx1.bounds().expect("expect non-empty polyline");
            let query_results = offset_loops_index.query_with_stack(
                bounds.min_x,
                bounds.min_y,
                bounds.max_x,
                bounds.max_y,
                &mut query_stack,
            );

            for j in query_results {
                if i == j {
                    // skip same index (no self intersects among the offset loops)
                    continue;
                }

                if visited_loop_pairs.contains(&(j, i)) {
                    // skip reversed index order (would end up comparing the same loops in another
                    // iteration)
                    continue;
                }

                visited_loop_pairs.insert((i, j));

                let loop2 = Self::get_loop(j, ccw_offset_loops, cw_offset_loops);

                let intrs_opts = FindIntersectsOptions {
                    pline1_aabb_index: Some(spatial_idx1),
                    pos_equal_eps,
                };

                let intersects = loop1
                    .indexed_pline
                    .polyline
                    .find_intersects_opt(&loop2.indexed_pline.polyline, &intrs_opts);

                if intersects.basic_intersects.is_empty()
                    && intersects.overlapping_intersects.is_empty()
                {
                    continue;
                }

                let mut slice_points = Vec::new();

                for intr in intersects.basic_intersects {
                    slice_points.push(intr);
                }

                // add overlapping start and end points
                for overlap_intr in intersects.overlapping_intersects {
                    let start_index1 = overlap_intr.start_index1;
                    let start_index2 = overlap_intr.start_index2;
                    slice_points.push(PlineBasicIntersect {
                        start_index1,
                        start_index2,
                        point: overlap_intr.point1,
                    });
                    slice_points.push(PlineBasicIntersect {
                        start_index1,
                        start_index2,
                        point: overlap_intr.point2,
                    });
                }

                let slice_point_set = SlicePointSet {
                    loop_idx1: i,
                    loop_idx2: j,
                    slice_points,
                };

                slice_point_sets.push(slice_point_set);
            }
        }

        slice_point_sets
    }

    /// **Step 3** of the multipolyline offset algorithm: Creates valid slices from intersection points.
    ///
    /// This method processes the intersection points from Step 2 to create polyline slices that
    /// represent valid portions of the offset geometry. Each offset loop is divided at intersection
    /// points, and the resulting slices are validated to ensure they maintain the correct distance
    /// from the original input polylines. Invalid slices (those that are too close to other input
    /// polylines) are filtered out.
    ///
    /// The slices are represented as `PlineViewData<T>` to avoid cloning the underlying polyline
    /// data, providing memory-efficient access to polyline segments.
    ///
    /// # Arguments
    ///
    /// * `ccw_offset_loops` - Counter-clockwise offset loops from Step 1
    /// * `cw_offset_loops` - Clockwise offset loops from Step 1
    /// * `slice_point_sets` - Intersection data from Step 2
    /// * `offset` - The offset distance used for validation
    /// * `options` - Offset options containing validation epsilons
    ///
    /// # Returns
    ///
    /// A vector of `DissectedSlice<T>` containing valid polyline slices that can be stitched
    /// together to form the final offset result. This includes valid offset polylines that had
    /// no intersection points (the entire polyline is preserved inside the `DissectedSlice<T>`).
    ///
    /// # Public Visibility
    ///
    /// This method is made public for visualization and testing purposes, allowing individual
    /// slices to be displayed and the slice validation logic to be independently tested.
    pub fn create_valid_slices_from_intersects(
        &self,
        ccw_offset_loops: &[OffsetLoop<T>],
        cw_offset_loops: &[OffsetLoop<T>],
        slice_point_sets: &[SlicePointSet<T>],
        offset: T,
        options: &ShapeOffsetOptions<T>,
    ) -> Vec<DissectedSlice<T>> {
        let offset_loop_count = ccw_offset_loops.len() + cw_offset_loops.len();
        let pos_equal_eps = options.pos_equal_eps;
        let offset_dist_eps = options.offset_dist_eps;

        let mut slice_points_lookup = BTreeMap::<usize, Vec<usize>>::new();
        for (set_idx, set) in slice_point_sets.iter().enumerate() {
            slice_points_lookup
                .entry(set.loop_idx1)
                .or_default()
                .push(set_idx);
            slice_points_lookup
                .entry(set.loop_idx2)
                .or_default()
                .push(set_idx);
        }

        let mut sorted_intrs = Vec::new();
        let mut slices_data = Vec::new();
        let mut query_stack = Vec::new();

        /// A point where an offset loop should be divided during slice creation.
        ///
        /// This structure represents a specific location on a polyline where an intersection
        /// occurs, defined by both the segment index and the exact position. These points
        /// are used to divide offset loops into valid slices.
        #[derive(Debug, Clone, Copy)]
        struct DissectionPoint<T> {
            /// Index of the polyline segment containing this point
            seg_idx: usize,
            /// Exact 2D position of the dissection point
            pos: Vector2<T>,
        }

        let create_slice = |pt1: &DissectionPoint<T>,
                            pt2: &DissectionPoint<T>,
                            offset_loop: &Polyline<T>|
         -> Option<PlineViewData<T>> {
            PlineViewData::from_slice_points(
                offset_loop,
                pt1.pos,
                pt1.seg_idx,
                pt2.pos,
                pt2.seg_idx,
                pos_equal_eps,
            )
        };

        let is_slice_valid = |v_data: &PlineViewData<T>,
                              offset_loop: &Polyline<T>,
                              parent_idx: usize,
                              query_stack: &mut Vec<usize>|
         -> bool {
            let slice_view = v_data.view(offset_loop);
            // ideally we don't want a segment created by the intersection point as it may be very
            // short with the midpoint essentially on top of the intersection point which leads to
            // the slice being considered valid when it shouldn't be (distance from intersection
            // point to polyline is always equal to the offset distance)
            //
            // to help with this we first check if we can use a segment that is not created by the
            // intersection point (index not at start or end of the slice)
            // if that's not possible then we check all both segments midpoints of the slice

            let vertex_count = slice_view.vertex_count();
            let (midpoint1, midpoint2) = if vertex_count > 3 {
                // if slice has more than 2 segments then we can use segment not created by
                // an intersection (arbitrarily picking segment from index 1 to index 2)
                (seg_midpoint(slice_view.at(1), slice_view.at(2)), None)
            } else if vertex_count == 3 {
                // if slice has exactly 3 points then we test both segment midpoints
                (
                    seg_midpoint(slice_view.at(0), slice_view.at(1)),
                    Some(seg_midpoint(slice_view.at(1), slice_view.at(2))),
                )
            } else {
                // if slice has only 2 points then we can only use the midpoint of the segment
                (seg_midpoint(slice_view.at(0), slice_view.at(1)), None)
            };

            // loop through input polylines and check if slice is too close (skipping parent
            // polyline since it's never too close)
            for input_loop_idx in
                (0..(self.ccw_plines.len() + self.cw_plines.len())).filter(|i| *i != parent_idx)
            {
                let parent_loop = if input_loop_idx < self.ccw_plines.len() {
                    &self.ccw_plines[input_loop_idx]
                } else {
                    &self.cw_plines[input_loop_idx - self.ccw_plines.len()]
                };

                if !point_valid_for_offset(
                    &parent_loop.polyline,
                    offset,
                    &parent_loop.spatial_index,
                    midpoint1,
                    query_stack,
                    pos_equal_eps,
                    offset_dist_eps,
                ) {
                    return false;
                }

                if let Some(midpoint2) = midpoint2
                    && !point_valid_for_offset(
                        &parent_loop.polyline,
                        offset,
                        &parent_loop.spatial_index,
                        midpoint2,
                        query_stack,
                        pos_equal_eps,
                        offset_dist_eps,
                    )
                {
                    return false;
                }
            }
            true
        };

        for loop_idx in 0..offset_loop_count {
            sorted_intrs.clear();
            let curr_loop = Self::get_loop(loop_idx, ccw_offset_loops, cw_offset_loops);

            if let Some(slice_point_set_idxs) = slice_points_lookup.get(&loop_idx) {
                // gather all the intersects for the current loop
                sorted_intrs.extend(slice_point_set_idxs.iter().flat_map(|set_idx| {
                    let set = &slice_point_sets[*set_idx];
                    debug_assert!(set.loop_idx1 == loop_idx || set.loop_idx2 == loop_idx);
                    let loop_is_first_index = set.loop_idx1 == loop_idx;
                    set.slice_points.iter().map(move |intr_pt| {
                        let seg_idx = if loop_is_first_index {
                            intr_pt.start_index1
                        } else {
                            intr_pt.start_index2
                        };
                        DissectionPoint {
                            seg_idx,
                            pos: intr_pt.point,
                        }
                    })
                }));

                // sort the intersect points along direction of polyline
                sorted_intrs.sort_unstable_by(|a, b| {
                    // sort by the segment index, then if both intersects on the same segment sort
                    // by distance from start of segment
                    a.seg_idx.cmp(&b.seg_idx).then_with(|| {
                        let seg_start = curr_loop.indexed_pline.polyline.at(a.seg_idx).pos();
                        let dist1 = dist_squared(a.pos, seg_start);
                        let dist2 = dist_squared(b.pos, seg_start);
                        dist1.total_cmp(&dist2)
                    })
                });

                // construct valid slices to later be stitched together
                if sorted_intrs.len() == 1 {
                    // treat whole loop as slice
                    let v_data =
                        PlineViewData::from_entire_pline(&curr_loop.indexed_pline.polyline);
                    if is_slice_valid(
                        &v_data,
                        &curr_loop.indexed_pline.polyline,
                        curr_loop.parent_loop_idx,
                        &mut query_stack,
                    ) {
                        slices_data.push(DissectedSlice {
                            source_idx: loop_idx,
                            v_data,
                        });
                    }
                } else {
                    // create slices from adjacent points
                    let mut windows = sorted_intrs.windows(2);
                    while let Some([pt1, pt2]) = windows.next() {
                        if let Some(v_data) =
                            create_slice(pt1, pt2, &curr_loop.indexed_pline.polyline)
                            && is_slice_valid(
                                &v_data,
                                &curr_loop.indexed_pline.polyline,
                                curr_loop.parent_loop_idx,
                                &mut query_stack,
                            )
                        {
                            slices_data.push(DissectedSlice {
                                source_idx: loop_idx,
                                v_data,
                            });
                        }
                    }

                    // collect slice from last to start
                    let pt1 = sorted_intrs.last().unwrap();
                    let pt2 = &sorted_intrs[0];
                    if let Some(v_data) = create_slice(pt1, pt2, &curr_loop.indexed_pline.polyline)
                        && is_slice_valid(
                            &v_data,
                            &curr_loop.indexed_pline.polyline,
                            curr_loop.parent_loop_idx,
                            &mut query_stack,
                        )
                    {
                        slices_data.push(DissectedSlice {
                            source_idx: loop_idx,
                            v_data,
                        });
                    }
                }
            } else {
                // no intersects but still must test distance of one vertex position since it may be
                // inside another offset (completely eclipsed by island offset)
                let v_data = PlineViewData::from_entire_pline(&curr_loop.indexed_pline.polyline);
                if is_slice_valid(
                    &v_data,
                    &curr_loop.indexed_pline.polyline,
                    curr_loop.parent_loop_idx,
                    &mut query_stack,
                ) {
                    slices_data.push(DissectedSlice {
                        source_idx: loop_idx,
                        v_data,
                    });
                }
            }
        }

        slices_data
    }

    /// **Step 4** of the multipolyline offset algorithm: Stitches slices together into final shapes.
    ///
    /// This method takes the valid slices from Step 3 and connects them end-to-end to form
    /// complete offset polylines. It uses spatial indexing to efficiently find adjacent slice
    /// endpoints and stitches them together, handling both simple connections and complex
    /// cases where multiple slices need to be joined.
    ///
    /// The method processes each unvisited slice, following the chain of connected slices until
    /// a closed loop is formed or no more connections can be found. The resulting polylines
    /// are then classified by orientation and returned as part of a complete `Shape`.
    ///
    /// # Arguments
    ///
    /// * `slices_data` - Valid slices from Step 3 (consumed by this method)
    /// * `ccw_offset_loops` - Counter-clockwise offset loops of the shape (for slice source lookup)
    /// * `cw_offset_loops` - Clockwise offset loops of the shape (for slice source lookup)
    /// * `pos_equal_eps` - Epsilon for position equality when extending polylines
    /// * `slice_join_eps` - Epsilon for finding adjacent slice endpoints
    ///
    /// # Returns
    ///
    /// A complete `Shape` containing the final offset result with properly oriented polylines.
    ///
    /// # Public Visibility
    ///
    /// This method is made public for visualization and testing purposes, allowing the stitching
    /// process to be observed and the final assembly logic to be independently tested.
    pub fn stitch_slices_together(
        &self,
        slices_data: Vec<DissectedSlice<T>>,
        ccw_offset_loops: &[OffsetLoop<T>],
        cw_offset_loops: &[OffsetLoop<T>],
        pos_equal_eps: T,
        slice_join_eps: T,
    ) -> Self {
        if slices_data.is_empty() {
            return Self::empty();
        }

        let mut ccw_plines_result = Vec::new();
        let mut cw_plines_result = Vec::new();

        let slice_starts_aabb_index = {
            let mut builder = StaticAABB2DIndexBuilder::new(slices_data.len());
            for slice in slices_data.iter() {
                let start_point = slice.v_data.updated_start.pos();
                builder.add(
                    start_point.x - slice_join_eps,
                    start_point.y - slice_join_eps,
                    start_point.x + slice_join_eps,
                    start_point.y + slice_join_eps,
                );
            }
            builder.build().unwrap()
        };

        let mut visited_slices_idxs = vec![false; slices_data.len()];
        let mut query_results = Vec::new();
        let mut query_stack = Vec::new();

        for slice_idx in 0..slices_data.len() {
            if visited_slices_idxs[slice_idx] {
                continue;
            }
            visited_slices_idxs[slice_idx] = true;

            let mut current_index = slice_idx;
            let mut loop_count = 0;
            let max_loop_count = slices_data.len();
            let mut current_pline = Polyline::new();

            loop {
                if loop_count > max_loop_count {
                    // prevent infinite loop
                    unreachable!(
                        "loop_count exceeded max_loop_count while stitching slices together"
                    );
                }
                loop_count += 1;

                let curr_slice = &slices_data[current_index];
                let source_loop =
                    Self::get_loop(curr_slice.source_idx, ccw_offset_loops, cw_offset_loops);
                let slice_view = curr_slice.v_data.view(&source_loop.indexed_pline.polyline);
                let slice_userdata_values = slice_view.get_userdata_values();
                current_pline.extend_remove_repeat(&slice_view, pos_equal_eps);
                current_pline.add_userdata_values(slice_userdata_values);

                query_results.clear();
                let slice_end_point = curr_slice.v_data.end_point;
                let mut aabb_index_visitor = |i: usize| {
                    if !visited_slices_idxs[i] {
                        query_results.push(i);
                    }
                };
                slice_starts_aabb_index.visit_query_with_stack(
                    slice_end_point.x - slice_join_eps,
                    slice_end_point.y - slice_join_eps,
                    slice_end_point.x + slice_join_eps,
                    slice_end_point.y + slice_join_eps,
                    &mut aabb_index_visitor,
                    &mut query_stack,
                );

                if query_results.is_empty() {
                    if current_pline.vertex_count() > 2 {
                        current_pline.remove_last();
                        current_pline.set_is_closed(true);
                    }
                    let is_ccw = current_pline.orientation() == PlineOrientation::CounterClockwise;
                    if is_ccw {
                        ccw_plines_result.push(IndexedPolyline::new(current_pline));
                    } else {
                        cw_plines_result.push(IndexedPolyline::new(current_pline));
                    }
                    break;
                }

                current_index = query_results
                    .iter()
                    .find_map(|i| {
                        let slice = &slices_data[*i];
                        if slice.source_idx == curr_slice.source_idx {
                            Some(*i)
                        } else {
                            None
                        }
                    })
                    .unwrap_or_else(|| query_results[0]);

                visited_slices_idxs[current_index] = true;
            }
        }

        let plines_index = {
            let mut b =
                StaticAABB2DIndexBuilder::new(ccw_plines_result.len() + cw_plines_result.len());

            let mut add_all_bounds = |plines: &[IndexedPolyline<T>]| {
                for pline in plines.iter() {
                    let bounds = pline
                        .spatial_index
                        .bounds()
                        .expect("expect non-empty polyline");

                    b.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
                }
            };

            add_all_bounds(&ccw_plines_result);
            add_all_bounds(&cw_plines_result);

            b.build().unwrap()
        };

        Shape {
            ccw_plines: ccw_plines_result,
            cw_plines: cw_plines_result,
            plines_index,
        }
    }

    fn get_loop<'a>(
        i: usize,
        s1: &'a [OffsetLoop<T>],
        s2: &'a [OffsetLoop<T>],
    ) -> &'a OffsetLoop<T> {
        if i < s1.len() {
            &s1[i]
        } else {
            &s2[i - s1.len()]
        }
    }

    /// Perform a boolean operation between `self` and `other` producing a new `Shape<T>`.
    ///
    /// Shape booleans use the same signed-loop convention as `Shape`: CCW loops add filled
    /// area, CW loops subtract hole area, and point membership is the non-zero sum of loop
    /// winding numbers. Geometric clipping is delegated to [`PlineSource::boolean`]; this
    /// method adds the shape-level bookkeeping needed to preserve islands, holes, and
    /// untouched loops across multiple input loops.
    ///
    /// Open polylines are treated as zero-area linework. They are clipped according to the
    /// requested set operation and then clipped again against the final filled area so linework
    /// does not remain visible inside material that already covers it.
    ///
    /// The implementation uses the same regularized polygon-set model described by Martinez,
    /// Rueda, and Feito (2009), Vatti (1992), and Greiner and Hormann (1998). Degenerate
    /// boundary cases such as shared edges, tangencies, and hole-boundary intersections are handled
    /// explicitly because later work on degenerate polygon clipping, for example Liu, Hormann, and
    /// Loop (2019), shows that vertex-only containment probes are not reliable for those cases.
    ///
    /// # References
    ///
    /// - F. Martinez, A. J. Rueda, and F. R. Feito, "A new algorithm for computing Boolean
    ///   operations on polygons", *Computers & Geosciences* 35(6), 1177-1185, 2009. DOI:
    ///   `10.1016/j.cageo.2008.08.009`.
    /// - B. R. Vatti, "A generic solution to polygon clipping", *Communications of the ACM*
    ///   35(7), 56-63, 1992. DOI: `10.1145/129902.129906`.
    /// - G. Greiner and K. Hormann, "Efficient clipping of arbitrary polygons", *ACM Transactions
    ///   on Graphics* 17(2), 71-83, 1998. DOI: `10.1145/274363.274364`.
    /// - F. Liu, K. Hormann, and C. Loop, "Clipping simple polygons with degenerate
    ///   intersections", *Computers & Graphics: X* 2, 100007, 2019. DOI:
    ///   `10.1016/j.cagx.2019.100007`.
    ///
    /// The `op` can be `BooleanOp::Or`, `BooleanOp::And`, `BooleanOp::Not`, or `BooleanOp::Xor`.
    pub fn boolean(&self, other: &Self, op: BooleanOp) -> Self {
        if self.has_open_plines() || other.has_open_plines() {
            return self.boolean_with_open_plines(other, op);
        }

        if let Some(canonical_self) = self.canonicalize_standalone_cw_loops() {
            return canonical_self.boolean(other, op);
        }

        if let Some(canonical_other) = other.canonicalize_standalone_cw_loops() {
            return self.boolean(&canonical_other, op);
        }

        if self.is_empty_shape() {
            return match op {
                BooleanOp::Or | BooleanOp::Xor => other.clone(),
                BooleanOp::And | BooleanOp::Not => Self::empty(),
            };
        }

        if other.is_empty_shape() {
            return match op {
                BooleanOp::Or | BooleanOp::Xor | BooleanOp::Not => self.clone(),
                BooleanOp::And => Self::empty(),
            };
        }

        if self.same_loop_bins(other) {
            // Set identities avoid feeding identical signed loops through the pairwise collector,
            // where equal CW holes can otherwise re-enter the result as positive material.
            return match op {
                BooleanOp::Or | BooleanOp::And => self.clone(),
                BooleanOp::Not | BooleanOp::Xor => Self::empty(),
            };
        }

        if matches!(op, BooleanOp::Or)
            && (!self.cw_plines.is_empty() || !other.cw_plines.is_empty())
            && self.ccw_plines.len() == 1
            && other.ccw_plines.len() == 1
        {
            return self.boolean_or(other);
        }

        if self.ccw_plines.len() == 1
            && other.ccw_plines.len() == 1
            && (!self.cw_plines.is_empty() || !other.cw_plines.is_empty())
        {
            if matches!(op, BooleanOp::And) {
                return self.boolean_and_single_shells(other);
            }

            if matches!(op, BooleanOp::Not) {
                return self.boolean_not_single_shells(other);
            }

            if matches!(op, BooleanOp::Xor) {
                return self.boolean_xor_single_shells(other);
            }
        }

        if matches!(op, BooleanOp::Xor)
            && self.cw_plines.is_empty()
            && other.cw_plines.is_empty()
            && self.ccw_plines.len() == 1
            && other.ccw_plines.len() == 1
        {
            let result = self.ccw_plines[0]
                .polyline
                .boolean(&other.ccw_plines[0].polyline, BooleanOp::Xor);
            let has_inverted_positive_piece = result
                .pos_plines
                .iter()
                .any(|rp| rp.pline.orientation() == PlineOrientation::Clockwise);
            if !matches!(result.result_info, BooleanResultInfo::Intersected)
                || !has_inverted_positive_piece
                || result.pos_plines.len() <= 2
            {
                // Preserve the regularized shape-level handling for disjoint, tangent, contained,
                // identical, and fully overlapping loops. The direct lower-level XOR path is only
                // needed for intersecting arc-heavy contours where composing two differences
                // through a later union can lose a valid piece.
            } else {
                let final_ccw = result
                    .pos_plines
                    .into_iter()
                    .map(|rp| Self::normalize_ccw(rp.pline))
                    .collect::<Vec<_>>();
                let final_cw = result
                    .neg_plines
                    .into_iter()
                    .map(|rp| Self::normalize_cw(rp.pline))
                    .collect::<Vec<_>>();

                return Self::build_from_signed_plines(final_ccw, final_cw);
            }
        }

        if matches!(op, BooleanOp::Xor) {
            // Set identity: A xor B == (A \ B) union (B \ A). Keeping XOR as a
            // composition means the hole-aware difference and union paths define the topology.
            let left_difference = self.boolean(other, BooleanOp::Not);
            let right_difference = other.boolean(self, BooleanOp::Not);
            return if left_difference.is_empty_shape() {
                right_difference
            } else if right_difference.is_empty_shape() {
                left_difference
            } else {
                left_difference.boolean(&right_difference, BooleanOp::Or)
            };
        }

        if matches!(op, BooleanOp::And) {
            let mut positive_pieces = Vec::new();
            let area_eps = Self::shape_boolean_area_epsilon();
            let mut query_stack = Vec::new();

            // Intersect only the filled regions first. Holes from either operand are subtracted
            // afterward, which avoids treating a hole boundary as if it were positive material.
            for ip in self.ccw_plines.iter() {
                let b1 = match ip.spatial_index.bounds() {
                    Some(bb) => bb,
                    None => continue,
                };
                let mut candidate_visitor = |j: usize| {
                    let jp = &other.ccw_plines[j];
                    let pline_result = ip.polyline.boolean(&jp.polyline, BooleanOp::And);
                    for rp in pline_result.pos_plines {
                        if rp.pline.area().abs() <= area_eps {
                            continue;
                        }

                        positive_pieces.push(Self::normalize_ccw(rp.pline));
                    }
                };
                Self::visit_loop_candidates(
                    other,
                    &b1,
                    true,
                    &mut query_stack,
                    &mut candidate_visitor,
                );
            }

            let mut result =
                Self::build_from_signed_plines(Self::merge_ccw_plines(positive_pieces), Vec::new());

            for hole in self.cw_plines.iter().chain(other.cw_plines.iter()) {
                let positive_hole = Polyline::create_from(&PlineInversionView::new(&hole.polyline));
                result = result.boolean(&Self::from_plines([positive_hole]), BooleanOp::Not);
            }

            return result;
        }

        // Collect pairwise lower-level boolean results. CW loops are passed through
        // PlineInversionView because the lower polyline boolean operates on positive-area loops.
        let mut all_results = Vec::new();

        let loop_center = |pline: &Polyline<T>| {
            let extents = pline.extents().expect("expected non-empty polyline");
            Vector2::new(
                (extents.min_x + extents.max_x) / T::two(),
                (extents.min_y + extents.max_y) / T::two(),
            )
        };

        let shape_contains_point = |shape: &Self, point: Vector2<T>| {
            shape
                .ccw_plines
                .iter()
                .map(|ip| ip.polyline.winding_number(point))
                .chain(
                    shape
                        .cw_plines
                        .iter()
                        .map(|ip| ip.polyline.winding_number(point)),
                )
                .sum::<i32>()
                != 0
        };

        // Probe several interior candidates when resolving containment-style lower-level results.
        // A single vertex is unreliable for edge-sharing and hole-boundary cases because it can
        // lie exactly on a boundary where winding-number membership is intentionally ambiguous.
        let shape_overlaps_pline_area = |shape: &Self, pline: &Polyline<T>| {
            shape_contains_point(shape, loop_center(pline))
                || pline
                    .iter_vertexes()
                    .any(|v| shape_contains_point(shape, v.pos()))
                || pline
                    .iter_segments()
                    .any(|(v1, v2)| shape_contains_point(shape, seg_midpoint(v1, v2)))
        };

        fn pline_area_contains_pline_area<T, S>(container: &S, candidate: &Polyline<T>) -> bool
        where
            T: Real,
            S: PlineSource<Num = T> + ?Sized,
        {
            let extents = match candidate.extents() {
                Some(extents) => extents,
                None => return false,
            };
            let center = Vector2::new(
                (extents.min_x + extents.max_x) / T::two(),
                (extents.min_y + extents.max_y) / T::two(),
            );

            // Use center and segment-midpoint probes instead of a single vertex probe. Vertices
            // can lie exactly on a boundary in the cases that split holes or share edges.
            container.winding_number(center) != 0
                && candidate
                    .iter_segments()
                    .all(|(v1, v2)| container.winding_number(seg_midpoint(v1, v2)) != 0)
        }

        // Track which input loops participated in a non-disjoint lower-level result. Unused-loop
        // retention is what keeps distant islands and untouched holes in union/difference output.
        let mut self_used_ccw = vec![false; self.ccw_plines.len()];
        let mut self_used_cw = vec![false; self.cw_plines.len()];
        let mut othr_used_ccw = vec![false; other.ccw_plines.len()];
        let mut othr_used_cw = vec![false; other.cw_plines.len()];
        let mut query_stack = Vec::new();

        // For each loop in self vs each loop in other, run the lower-level boolean operation.
        // The four signed-loop pairings need separate handling because holes are subtractive at
        // shape level but are inverted to positive contours for the lower-level boolean.

        // 1) ccw_plines vs ccw_plines
        for (i, ip) in self.ccw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            let mut candidate_visitor = |j: usize| {
                let jp = &other.ccw_plines[j];
                let result = ip.polyline.boolean(&jp.polyline, op);
                let skip_result = (matches!(op, BooleanOp::And)
                    && ((matches!(result.result_info, BooleanResultInfo::Pline1InsidePline2)
                        && !shape_overlaps_pline_area(other, &ip.polyline))
                        || (matches!(result.result_info, BooleanResultInfo::Pline2InsidePline1)
                            && !shape_overlaps_pline_area(self, &jp.polyline))))
                    || (matches!(op, BooleanOp::Not)
                        && matches!(result.result_info, BooleanResultInfo::Pline2InsidePline1)
                        && !shape_overlaps_pline_area(self, &jp.polyline))
                    || (matches!(op, BooleanOp::Not)
                        && matches!(result.result_info, BooleanResultInfo::Pline1InsidePline2)
                        && !shape_overlaps_pline_area(other, &ip.polyline));

                // If the operation had any real relationship, mark both loops as used. This
                // matters for empty overlapping results such as A - A and A xor A.
                if !skip_result && !matches!(result.result_info, BooleanResultInfo::Disjoint) {
                    let mut mark_self = true;
                    let mut mark_other = true;

                    if matches!(op, BooleanOp::Or) {
                        if matches!(result.result_info, BooleanResultInfo::Pline1InsidePline2)
                            && !shape_overlaps_pline_area(other, &ip.polyline)
                        {
                            mark_self = false;
                        }

                        if matches!(result.result_info, BooleanResultInfo::Pline2InsidePline1)
                            && !shape_overlaps_pline_area(self, &jp.polyline)
                        {
                            mark_other = false;
                        }
                    }

                    if mark_self {
                        self_used_ccw[i] = true;
                    }
                    if mark_other {
                        othr_used_ccw[j] = true;
                    }
                }

                if !skip_result && (!result.pos_plines.is_empty() || !result.neg_plines.is_empty())
                {
                    all_results.push(result);
                }
            };
            Self::visit_loop_candidates(other, &b1, true, &mut query_stack, &mut candidate_visitor);
        }

        // 2) ccw_plines vs cw_plines
        for (i, ip) in self.ccw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };

            let mut candidate_visitor = |j: usize| {
                let jp = &other.cw_plines[j];
                let jp_inverted = PlineInversionView::new(&jp.polyline);
                let result = if matches!(op, BooleanOp::Not) {
                    ip.polyline.boolean(&jp_inverted, BooleanOp::And)
                } else {
                    ip.polyline.boolean(&jp_inverted, op)
                };
                // Unioning an island fully inside a hole should keep the positive island
                // and the hole. The lower-level contour union sees two positive contours and
                // can return the hole outline as positive material, so skip that pairwise
                // result and let unused-loop retention keep the two loops separately.
                let self_inside_other_hole = matches!(op, BooleanOp::Or)
                    && pline_area_contains_pline_area(&jp_inverted, &ip.polyline);
                let skip_result = if matches!(op, BooleanOp::Not) {
                    !self_used_ccw[i] || matches!(result.result_info, BooleanResultInfo::Disjoint)
                } else if self_inside_other_hole {
                    true
                } else {
                    (matches!(op, BooleanOp::Or | BooleanOp::And))
                        && matches!(result.result_info, BooleanResultInfo::Pline1InsidePline2)
                };

                if !matches!(op, BooleanOp::Not)
                    && !skip_result
                    && !matches!(result.result_info, BooleanResultInfo::Disjoint)
                {
                    self_used_ccw[i] = true;
                    othr_used_cw[j] = true;
                }

                if !skip_result && (!result.pos_plines.is_empty() || !result.neg_plines.is_empty())
                {
                    all_results.push(result);
                }
            };
            Self::visit_loop_candidates(
                other,
                &b1,
                false,
                &mut query_stack,
                &mut candidate_visitor,
            );
        }

        // 3) cw_plines vs ccw_plines
        for (i, ip) in self.cw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            let ip_inverted = PlineInversionView::new(&ip.polyline);

            let mut candidate_visitor = |j: usize| {
                let jp = &other.ccw_plines[j];
                let result = ip_inverted.boolean(&jp.polyline, op);
                // Same island-in-hole rule as the ccw-vs-cw case, with operands swapped.
                let other_inside_self_hole = matches!(op, BooleanOp::Or)
                    && pline_area_contains_pline_area(&ip_inverted, &jp.polyline);
                let skip_result = matches!(op, BooleanOp::Not)
                    || other_inside_self_hole
                    || (matches!(op, BooleanOp::Or | BooleanOp::And)
                        && matches!(result.result_info, BooleanResultInfo::Pline2InsidePline1));

                if !skip_result && !matches!(result.result_info, BooleanResultInfo::Disjoint) {
                    self_used_cw[i] = true;
                    othr_used_ccw[j] = true;
                }

                if !skip_result && (!result.pos_plines.is_empty() || !result.neg_plines.is_empty())
                {
                    all_results.push(result);
                }
            };
            Self::visit_loop_candidates(other, &b1, true, &mut query_stack, &mut candidate_visitor);
        }

        // 4) cw_plines vs cw_plines
        for (i, ip) in self.cw_plines.iter().enumerate() {
            let b1 = match ip.spatial_index.bounds() {
                Some(bb) => bb,
                None => continue,
            };
            let ip_inverted = PlineInversionView::new(&ip.polyline);

            let mut candidate_visitor = |j: usize| {
                let jp = &other.cw_plines[j];
                let jp_inverted = PlineInversionView::new(&jp.polyline);
                let result = ip_inverted.boolean(&jp_inverted, op);

                if !matches!(result.result_info, BooleanResultInfo::Disjoint) {
                    self_used_cw[i] = true;
                    othr_used_cw[j] = true;
                }

                if !result.pos_plines.is_empty() || !result.neg_plines.is_empty() {
                    all_results.push(result);
                }
            };
            Self::visit_loop_candidates(
                other,
                &b1,
                false,
                &mut query_stack,
                &mut candidate_visitor,
            );
        }

        // At this point, pairwise BooleanResult values are normalized back into shape bins.
        // Positive result loops become CCW material; negative result loops become CW holes.
        let mut final_ccw = Vec::new();
        let mut final_cw = Vec::new();

        let add_ccw = |pline: Polyline<T>, final_ccw: &mut Vec<Polyline<T>>| {
            final_ccw.push(Self::normalize_ccw(pline));
        };

        let add_cw = |pline: Polyline<T>, final_cw: &mut Vec<Polyline<T>>| {
            final_cw.push(Self::normalize_cw(pline));
        };

        for boolean_result in all_results {
            // Each lower-level result can contain positive material or negative holes; normalize
            // them back into shape bins before retaining untouched input loops.
            for rp in boolean_result.pos_plines {
                add_ccw(rp.pline, &mut final_ccw);
            }
            for rp in boolean_result.neg_plines {
                add_cw(rp.pline, &mut final_cw);
            }
        }

        if matches!(op, BooleanOp::Or) {
            final_ccw = Self::merge_ccw_plines(final_ccw);
        }

        // Add input loops that never participated in a non-disjoint pairwise operation.
        match op {
            BooleanOp::Or | BooleanOp::Xor => {
                for (i, used) in self_used_ccw.iter().enumerate() {
                    if !used {
                        add_ccw(self.ccw_plines[i].polyline.clone(), &mut final_ccw);
                    }
                }
                for (i, used) in self_used_cw.iter().enumerate() {
                    if !used {
                        add_cw(self.cw_plines[i].polyline.clone(), &mut final_cw);
                    }
                }
                for (j, used) in othr_used_ccw.iter().enumerate() {
                    if !used {
                        add_ccw(other.ccw_plines[j].polyline.clone(), &mut final_ccw);
                    }
                }
                for (j, used) in othr_used_cw.iter().enumerate() {
                    if !used {
                        add_cw(other.cw_plines[j].polyline.clone(), &mut final_cw);
                    }
                }
            }
            BooleanOp::And => {
                // For intersection, untouched loops have no overlapping area to contribute.
            }
            BooleanOp::Not => {
                // For difference, only untouched loops from the left-hand shape remain.
                for (i, used) in self_used_ccw.iter().enumerate() {
                    if !used {
                        add_ccw(self.ccw_plines[i].polyline.clone(), &mut final_ccw);
                    }
                }
                for (i, used) in self_used_cw.iter().enumerate() {
                    if !used {
                        add_cw(self.cw_plines[i].polyline.clone(), &mut final_cw);
                    }
                }
            }
        }

        if matches!(op, BooleanOp::Or) {
            final_ccw = Self::merge_ccw_plines(final_ccw);
        }

        let area_eps = Self::shape_boolean_area_epsilon();
        final_ccw.retain(|pline| pline.area().abs() > area_eps);
        final_cw.retain(|pline| pline.area().abs() > area_eps);

        if !final_cw.is_empty() {
            // Holes can overlap after multiple pairwise differences. Merge them by temporarily
            // treating holes as positive filled regions, then invert the merged results back.
            let positive_holes = final_cw
                .into_iter()
                .map(Self::normalize_ccw)
                .collect::<Vec<_>>();

            final_cw = Self::merge_ccw_plines(positive_holes)
                .into_iter()
                .map(Self::normalize_cw)
                .collect();
        }

        Self::build_from_signed_plines(final_ccw, final_cw)
    }

    /// Union
    pub fn union(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Or)
    }

    /// Intersection
    pub fn intersection(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::And)
    }

    /// Difference: shape1 minus shape2
    pub fn difference(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Not)
    }

    /// Symmetric difference (xor)
    pub fn xor(&self, other: &Self) -> Self {
        self.boolean(other, BooleanOp::Xor)
    }

    /// Translates all polylines by `(dx, dy)`.
    pub fn translate_mut(&mut self, dx: T, dy: T) {
        for loop_poly in &mut self.ccw_plines {
            loop_poly.polyline.translate_mut(dx, dy);
            // rebuild each loop's index
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            loop_poly.polyline.translate_mut(dx, dy);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        // rebuild the shape’s overall index
        self.plines_index = self.build_plines_index();
    }

    /// Scales all polylines by `scale_factor` about the origin `(0, 0)`.
    /// (You might want an overload that takes an `origin` parameter.)
    pub fn scale_mut(&mut self, scale_factor: T) {
        for loop_poly in &mut self.ccw_plines {
            loop_poly.polyline.scale_mut(scale_factor);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            loop_poly.polyline.scale_mut(scale_factor);
            loop_poly.spatial_index = loop_poly.polyline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Rotates all polylines by `theta` radians about `(0, 0)`.
    /// (You might want an overload for a custom center.)
    pub fn rotate_mut(&mut self, theta: T) {
        // We treat rotation as: (x, y) -> (x cos θ – y sin θ, x sin θ + y cos θ)
        // Each polyline's `translate_mut` or manual iteration can do that.
        let cos_theta = theta.cos();
        let sin_theta = theta.sin();
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                let rx = px * cos_theta - py * sin_theta;
                let ry = px * sin_theta + py * cos_theta;
                v.x = rx;
                v.y = ry;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                let rx = px * cos_theta - py * sin_theta;
                let ry = px * sin_theta + py * cos_theta;
                v.x = rx;
                v.y = ry;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Mirrors (reflects) all polylines about the X-axis (example).
    /// For a more general "mirror across a line," you can parameterize the reflection formula.
    /// This example just flips `y -> -y`.
    pub fn mirror_x_mut(&mut self) {
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                // reflect y about the x-axis
                v.y = -v.y;
                // if you want to invert bulge as well, do so if it's an arc:
                // (since reflection can flip arc orientation)
                v.bulge = -v.bulge;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                v.y = -v.y;
                v.bulge = -v.bulge;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        self.plines_index = self.build_plines_index();
    }

    /// Re-centers the shape to place its bounding box center at the origin (0, 0).
    pub fn center_mut(&mut self) {
        if let Some(shape_aabb) = self.plines_index.bounds() {
            let cx = (shape_aabb.min_x + shape_aabb.max_x) / T::two();
            let cy = (shape_aabb.min_y + shape_aabb.max_y) / T::two();
            // Translate the shape so that bounding box center -> (0, 0)
            self.translate_mut(-cx, -cy);
        }
    }

    /// Applies a full 2D transform if you want a custom matrix approach.
    /// transform matrix is `[ [a, b], [c, d] ]` plus a translation `(tx, ty)`.
    /// the mapping is: (x, y) -> (a x + b y + tx, c x + d y + ty).
    pub fn transform_mut(
        &mut self,
        scale_x: T,
        scale_y: T,
        rot_cos: T,
        rot_sin: T,
        trans_x: T,
        trans_y: T,
    ) {
        // apply transform to polylines
        for loop_poly in &mut self.ccw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                v.x = scale_x * px + scale_y * py + trans_x;
                v.y = rot_cos * px + rot_sin * py + trans_y;
                // If arcs are used, you might need more logic to handle orientation, etc.
                // But for simple transform, bulge isn't changed, though reflection inverts sign, etc.
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }
        for loop_poly in &mut self.cw_plines {
            let pline = &mut loop_poly.polyline;
            for i in 0..pline.vertex_count() {
                let mut v = pline.at(i);
                let (px, py) = (v.x, v.y);
                v.x = scale_x * px + scale_y * py + trans_x;
                v.y = rot_cos * px + rot_sin * py + trans_y;
                pline.set_vertex(i, v);
            }
            loop_poly.spatial_index = pline.create_aabb_index();
        }

        // now rebuild the shape's top-level index
        self.plines_index = self.build_plines_index();
    }

    /// Helper to rebuild the shape’s own bounding index after polylines are mutated.
    fn build_plines_index(&self) -> StaticAABB2DIndex<T> {
        let total_len = self.ccw_plines.len() + self.cw_plines.len();
        let mut builder = StaticAABB2DIndexBuilder::new(total_len);
        for loop_poly in &self.ccw_plines {
            let bounds = loop_poly
                .spatial_index
                .bounds()
                .expect("empty pline unexpected");
            builder.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
        }
        for loop_poly in &self.cw_plines {
            let bounds = loop_poly
                .spatial_index
                .bounds()
                .expect("empty pline unexpected");
            builder.add(bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y);
        }
        builder.build().unwrap()
    }
}

impl<T: Real> ShapeTransformBuilder<T> {
    /// Creates a builder from an existing shape (consumes the shape).
    pub fn new(shape: Shape<T>) -> Self {
        ShapeTransformBuilder {
            shape,
            scale_factor: T::one(),
            rotate_angle: T::zero(),
            rotate_anchor: None,
            translate_x: T::zero(),
            translate_y: T::zero(),
            invert_direction: false,
        }
    }

    /// Apply an additional uniform scale factor.
    pub fn scale_mut(&mut self, factor: T) -> &mut Self {
        self.scale_factor = self.scale_factor * factor;
        self
    }

    /// Translate by (dx, dy).
    pub fn translate_mut(&mut self, dx: T, dy: T) -> &mut Self {
        self.translate_x = self.translate_x + dx;
        self.translate_y = self.translate_y + dy;
        self
    }

    /// Toggle direction inversion for every polyline in the final shape.
    pub fn invert_direction_mut(&mut self) -> &mut Self {
        self.invert_direction = !self.invert_direction;
        self
    }

    /// Add a rotation about an anchor point.
    pub fn rotate_about_mut(&mut self, anchor: Vector2<T>, angle: T) -> &mut Self {
        self.rotate_angle = self.rotate_angle + angle;
        self.rotate_anchor = Some((anchor.x, anchor.y));
        self
    }

    /// Final build step: apply all transformations in the recorded order
    /// to each polyline, then rebuild the shape-level indexes once.
    pub fn build(mut self) -> Shape<T> {
        let do_invert = self.invert_direction;
        let s = self.scale_factor;
        let angle = self.rotate_angle;
        let (rc, rs) = (angle.cos(), angle.sin());
        let (tx, ty) = (self.translate_x, self.translate_y);

        let anchor = match self.rotate_anchor {
            None => (T::zero(), T::zero()),
            Some(a) => a,
        };

        // Apply transforms in a fixed order: invert direction, scale, rotate, then translate.
        for ip in &mut self.shape.ccw_plines {
            let p = &mut ip.polyline;
            if do_invert {
                p.invert_direction_mut();
            }
            for i in 0..p.vertex_count() {
                let v = p.at(i);

                let (mut x, mut y) = (v.x, v.y);

                // scale
                x = x * s;
                y = y * s;

                // rotate about anchor
                // translate so anchor is origin => rotate => move back
                x = x - anchor.0;
                y = y - anchor.1;
                let rx = x * rc - y * rs;
                let ry = x * rs + y * rc;
                x = rx + anchor.0;
                y = ry + anchor.1;

                // translate
                x = x + tx;
                y = y + ty;

                p.set(i, x, y, v.bulge);
            }
            ip.spatial_index = p.create_aabb_index();
        }
        for ip in &mut self.shape.cw_plines {
            let p = &mut ip.polyline;
            if do_invert {
                p.invert_direction_mut();
            }
            for i in 0..p.vertex_count() {
                let v = p.at(i);
                let (mut x, mut y) = (v.x, v.y);
                // scale
                x = x * s;
                y = y * s;
                // rotate about anchor
                x = x - anchor.0;
                y = y - anchor.1;
                let rx = x * rc - y * rs;
                let ry = x * rs + y * rc;
                x = rx + anchor.0;
                y = ry + anchor.1;
                // translate
                x = x + tx;
                y = y + ty;
                p.set(i, x, y, v.bulge);
            }
            ip.spatial_index = p.create_aabb_index();
        }

        // now rebuild shape's top-level index
        let mut builder =
            StaticAABB2DIndexBuilder::new(self.shape.ccw_plines.len() + self.shape.cw_plines.len());
        for ip in &self.shape.ccw_plines {
            if let Some(b) = ip.spatial_index.bounds() {
                builder.add(b.min_x, b.min_y, b.max_x, b.max_y);
            }
        }
        for ip in &self.shape.cw_plines {
            if let Some(b) = ip.spatial_index.bounds() {
                builder.add(b.min_x, b.min_y, b.max_x, b.max_y);
            }
        }
        self.shape.plines_index = builder.build().unwrap();

        self.shape
    }
}

/// Intersection data between two offset loops.
///
/// This structure contains all intersection points found between a pair of offset loops,
/// including both basic intersections and overlapping segment intersections. The data
/// is used to create slices by dividing the offset loops at these intersection points.
///
/// # Public Visibility
///
/// This struct is made public for visualization and testing purposes, allowing
/// intersection data to be inspected and displayed during algorithm execution.
#[derive(Debug, Clone)]
pub struct SlicePointSet<T> {
    /// Index of the first offset loop in the intersection pair
    pub loop_idx1: usize,
    /// Index of the second offset loop in the intersection pair
    pub loop_idx2: usize,
    /// All intersection points between the two loops
    pub slice_points: Vec<PlineBasicIntersect<T>>,
}

/// A validated slice of an offset polyline ready for stitching.
///
/// This structure represents a portion of an offset loop that has been validated
/// to maintain the correct distance from the original input polylines. The slice
/// is represented as a view into the source polyline to avoid unnecessary copying.
///
/// # Public Visibility
///
/// This struct is made public for visualization and testing purposes, allowing
/// individual slices to be displayed before the final stitching step.
#[derive(Debug, Clone, Copy)]
pub struct DissectedSlice<T: Real> {
    /// Index of the source offset loop this slice comes from
    pub source_idx: usize,
    /// View data defining the slice boundaries within the source polyline
    pub v_data: PlineViewData<T>,
}
