//! Shared shape-boolean fuzz helpers.
//!
//! The fuzz targets intentionally generate small but relationship-rich inputs, then assert the
//! same invariants as the integration tests: valid signed shape bins, fresh spatial indexes, and
//! sampled set-membership semantics. The byte reader returns deterministic defaults when input is
//! exhausted so minimized cases remain short and replayable.

#![allow(dead_code)]

use cavalier_contours::core::{math::Vector2, traits::FuzzyEq};
use cavalier_contours::polyline::{
    BooleanOp, PlineInversionView, PlineOrientation, PlineSource, PlineSourceMut, Polyline,
};
use cavalier_contours::shape_algorithms::Shape;
use std::f64::consts::PI;

const EPS: f64 = 1e-7;

/// Deterministic byte reader for lightweight, shrinkable fuzz input decoding.
pub struct ByteReader<'a> {
    data: &'a [u8],
    index: usize,
}

impl<'a> ByteReader<'a> {
    /// Create a reader over the current fuzz input.
    pub fn new(data: &'a [u8]) -> Self {
        Self { data, index: 0 }
    }

    /// Read one byte, returning zero after the input is exhausted.
    pub fn byte(&mut self) -> u8 {
        let byte = self.data.get(self.index).copied().unwrap_or(0);
        self.index = self.index.saturating_add(1);
        byte
    }

    /// Decode a boolean from the next byte.
    pub fn bool(&mut self) -> bool {
        self.byte() & 1 == 1
    }

    /// Decode an integer in an inclusive range.
    pub fn usize_range(&mut self, min: usize, max: usize) -> usize {
        min + usize::from(self.byte()) % (max - min + 1)
    }

    /// Decode a bounded finite float from four bytes.
    pub fn f64_range(&mut self, min: f64, max: f64) -> f64 {
        let raw = u32::from_le_bytes([self.byte(), self.byte(), self.byte(), self.byte()]);
        let t = f64::from(raw) / f64::from(u32::MAX);
        min + (max - min) * t
    }
}

/// Decode one of the four boolean operations.
pub fn boolean_op(reader: &mut ByteReader<'_>) -> BooleanOp {
    match reader.byte() & 3 {
        0 => BooleanOp::Or,
        1 => BooleanOp::And,
        2 => BooleanOp::Not,
        _ => BooleanOp::Xor,
    }
}

/// Create a closed CCW rectangle.
pub fn rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(xmin, ymin, 0.0);
    pline.add(xmax, ymin, 0.0);
    pline.add(xmax, ymax, 0.0);
    pline.add(xmin, ymax, 0.0);
    pline
}

/// Generate a rectangle with bounded positive width and height.
pub fn rectangle_from_bytes(reader: &mut ByteReader<'_>) -> Polyline<f64> {
    let x = reader.f64_range(-64.0, 64.0);
    let y = reader.f64_range(-64.0, 64.0);
    let width = reader.f64_range(0.01, 32.0);
    let height = reader.f64_range(0.01, 32.0);
    rectangle(x, y, x + width, y + height)
}

/// Generate a multi-rectangle shape for broad-phase and unused-loop coverage.
pub fn rectangle_shape(reader: &mut ByteReader<'_>, max_rects: usize) -> Shape<f64> {
    let count = reader.usize_range(1, max_rects);
    Shape::from_plines((0..count).map(|_| rectangle_from_bytes(reader)))
}

/// Generate a single rectangular ring so fuzzing repeatedly exercises CW hole handling.
pub fn donut_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let x = reader.f64_range(-40.0, 40.0);
    let y = reader.f64_range(-40.0, 40.0);
    let width = reader.f64_range(4.0, 36.0);
    let height = reader.f64_range(4.0, 36.0);
    let margin_x = reader.f64_range(width * 0.1, width * 0.4);
    let margin_y = reader.f64_range(height * 0.1, height * 0.4);

    let outer = rectangle(x, y, x + width, y + height);
    let mut inner = rectangle(
        x + margin_x,
        y + margin_y,
        x + width - margin_x,
        y + height - margin_y,
    );
    inner.invert_direction_mut();
    Shape::from_plines([outer, inner])
}

/// Create a full circle represented by two half-circle bulge segments.
pub fn circle(center_x: f64, center_y: f64, radius: f64) -> Polyline<f64> {
    let mut pline = Polyline::new_closed();
    pline.add(center_x - radius, center_y, 1.0);
    pline.add(center_x + radius, center_y, 1.0);
    pline
}

/// Generate arc-bearing shapes, including full circles and capsule-like loops.
pub fn arc_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let count = reader.usize_range(1, 3);
    Shape::from_plines((0..count).map(|_| {
        let x = reader.f64_range(-48.0, 48.0);
        let y = reader.f64_range(-48.0, 48.0);
        let radius = reader.f64_range(0.1, 24.0);
        if reader.bool() {
            circle(x, y, radius)
        } else {
            let mut capsule = Polyline::new_closed();
            capsule.add(x - radius, y - radius * 0.5, 0.0);
            capsule.add(x + radius, y - radius * 0.5, reader.f64_range(-1.5, 1.5));
            capsule.add(x + radius, y + radius * 0.5, 0.0);
            capsule.add(x - radius, y + radius * 0.5, reader.f64_range(-1.5, 1.5));
            capsule
        }
    }))
}

/// Generate small star-like polygon loops from sorted polar angles.
pub fn polygon_shape(reader: &mut ByteReader<'_>) -> Shape<f64> {
    let count = reader.usize_range(1, 3);
    Shape::from_plines((0..count).map(|_| {
        let cx = reader.f64_range(-48.0, 48.0);
        let cy = reader.f64_range(-48.0, 48.0);
        let vertex_count = reader.usize_range(3, 8);
        let base_radius = reader.f64_range(0.25, 24.0);
        let mut pline = Polyline::new_closed();

        for i in 0..vertex_count {
            let angle = 2.0 * PI * (i as f64) / (vertex_count as f64);
            let radius = base_radius * reader.f64_range(0.55, 1.45);
            pline.add(cx + radius * angle.cos(), cy + radius * angle.sin(), 0.0);
        }

        pline
    }))
}

/// Optionally transform a shape before booleaning to exercise transformed indexes and geometry.
pub fn maybe_transform(shape: &mut Shape<f64>, reader: &mut ByteReader<'_>) {
    if reader.bool() {
        shape.translate_mut(reader.f64_range(-12.0, 12.0), reader.f64_range(-12.0, 12.0));
    }
    if reader.bool() {
        shape.scale_mut(reader.f64_range(0.1, 3.0));
    }
    if reader.bool() {
        shape.rotate_mut(reader.f64_range(-PI, PI));
    }
}

/// Mirror the shape membership convention used by the integration tests.
fn shape_winding_number(shape: &Shape<f64>, point: Vector2<f64>) -> i32 {
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
        .sum()
}

/// Return true when a point is inside any signed loop under non-zero winding semantics.
fn shape_contains(shape: &Shape<f64>, point: Vector2<f64>) -> bool {
    shape_winding_number(shape, point) != 0
}

/// Sum signed shape area so fuzz targets can reject non-finite output.
pub fn shape_signed_area(shape: &Shape<f64>) -> f64 {
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.area())
        .chain(shape.cw_plines.iter().map(|ip| ip.polyline.area()))
        .sum()
}

/// Assert reusable shape invariants after each generated boolean operation.
pub fn assert_shape_valid(shape: &Shape<f64>) {
    for ip in &shape.ccw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::CounterClockwise);
        assert_matching_bounds(&ip.polyline, ip.spatial_index.bounds());
    }
    for ip in &shape.cw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::Clockwise);
        assert_matching_bounds(&ip.polyline, ip.spatial_index.bounds());
    }

    assert!(shape_signed_area(shape).is_finite());
}

/// Validate one signed loop's orientation and numeric sanity.
fn assert_loop_valid(pline: &Polyline<f64>, expected_orientation: PlineOrientation) {
    assert!(pline.is_closed());
    assert_eq!(pline.orientation(), expected_orientation);
    assert!(pline.remove_repeat_pos(EPS).is_none());
    for vertex in pline.iter_vertexes() {
        assert!(vertex.x.is_finite());
        assert!(vertex.y.is_finite());
        assert!(vertex.bulge.is_finite());
    }
}

/// Ensure a loop's cached spatial index still matches its polyline extents.
fn assert_matching_bounds(
    pline: &Polyline<f64>,
    index_bounds: Option<cavalier_contours::static_aabb2d_index::AABB<f64>>,
) {
    let pline_extents = pline.extents().expect("non-empty closed loop");
    let index_bounds = index_bounds.expect("non-empty closed loop index");
    assert!(pline_extents.min_x.fuzzy_eq_eps(index_bounds.min_x, EPS));
    assert!(pline_extents.min_y.fuzzy_eq_eps(index_bounds.min_y, EPS));
    assert!(pline_extents.max_x.fuzzy_eq_eps(index_bounds.max_x, EPS));
    assert!(pline_extents.max_y.fuzzy_eq_eps(index_bounds.max_y, EPS));
}

/// Compute a sampling envelope that covers both inputs and the result.
fn combined_extents(
    shapes: &[&Shape<f64>],
) -> Option<cavalier_contours::static_aabb2d_index::AABB<f64>> {
    shapes
        .iter()
        .filter_map(|shape| shape.plines_index.bounds())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some(curr) => cavalier_contours::static_aabb2d_index::AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        })
}

/// Check sampled membership semantics for one generated boolean case.
///
/// This oracle intentionally samples interior-biased fractions instead of boundaries because
/// boundary winding semantics are not the fuzz target's responsibility.
pub fn assert_boolean_semantics(a: &Shape<f64>, b: &Shape<f64>, op: BooleanOp) {
    assert_shape_valid(a);
    assert_shape_valid(b);
    let result = a.boolean(b, op);
    assert_shape_valid(&result);

    let Some(extents) = combined_extents(&[a, b, &result]) else {
        return;
    };

    let width = (extents.max_x - extents.min_x).max(1.0);
    let height = (extents.max_y - extents.min_y).max(1.0);
    let fractions = [0.137, 0.311, 0.587, 0.829];

    for fx in fractions {
        for fy in fractions {
            let point = Vector2::new(extents.min_x + width * fx, extents.min_y + height * fy);
            let in_a = shape_contains(a, point);
            let in_b = shape_contains(b, point);
            let expected = match op {
                BooleanOp::Or => in_a || in_b,
                BooleanOp::And => in_a && in_b,
                BooleanOp::Not => in_a && !in_b,
                BooleanOp::Xor => in_a != in_b,
            };
            assert_eq!(shape_contains(&result, point), expected);
        }
    }
}

/// Fuzz `PlineInversionView` directly because shape holes depend on it for lower-level booleans.
pub fn assert_inversion_boolean(reader: &mut ByteReader<'_>) {
    let pline1 = if reader.bool() {
        rectangle_from_bytes(reader)
    } else {
        circle(
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(0.1, 24.0),
        )
    };
    let pline2 = if reader.bool() {
        rectangle_from_bytes(reader)
    } else {
        circle(
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(-32.0, 32.0),
            reader.f64_range(0.1, 24.0),
        )
    };
    let inverted = PlineInversionView::new(&pline1);
    let result = inverted.boolean(&pline2, boolean_op(reader));

    for result_pline in result
        .pos_plines
        .into_iter()
        .chain(result.neg_plines.into_iter())
    {
        for vertex in result_pline.pline.iter_vertexes() {
            assert!(vertex.x.is_finite());
            assert!(vertex.y.is_finite());
            assert!(vertex.bulge.is_finite());
        }
        assert!(result_pline.pline.area().is_finite());
    }
}
