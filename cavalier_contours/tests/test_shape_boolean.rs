mod test_utils;

use cavalier_contours::core::{math::Vector2, traits::FuzzyEq};
use cavalier_contours::polyline::{
    BooleanOp, PlineOrientation, PlineSource, PlineSourceMut, Polyline, seg_arc_radius_and_center,
};
use cavalier_contours::shape_algorithms::{Shape, ShapeOffsetOptions};
use cavalier_contours::{assert_fuzzy_eq, pline_closed, pline_open};
use std::f64::consts::PI;
use std::{env, fs, path::Path};
use test_utils::to_debug_json_str;

const SHAPE_TEST_EPS: f64 = 1e-7;

/// A small helper to verify bounding boxes.
fn assert_extents_fuzzy_eq(
    shape: &Shape<f64>,
    expected_min_x: f64,
    expected_min_y: f64,
    expected_max_x: f64,
    expected_max_y: f64,
    eps: f64,
) {
    let aabb = shape.plines_index.bounds().unwrap();
    let actual_min_x = aabb.min_x;
    let actual_min_y = aabb.min_y;
    let actual_max_x = aabb.max_x;
    let actual_max_y = aabb.max_y;
    assert!(
        actual_min_x.fuzzy_eq_eps(expected_min_x, eps),
        "min_x => expected: {expected_min_x}, got: {actual_min_x}"
    );
    assert!(
        actual_min_y.fuzzy_eq_eps(expected_min_y, eps),
        "min_y => expected: {expected_min_y}, got: {actual_min_y}"
    );
    assert!(
        actual_max_x.fuzzy_eq_eps(expected_max_x, eps),
        "max_x => expected: {expected_max_x}, got: {actual_max_x}"
    );
    assert!(
        actual_max_y.fuzzy_eq_eps(expected_max_y, eps),
        "max_y => expected: {expected_max_y}, got: {actual_max_y}"
    );
}

/// Create a simple rectangle polyline oriented along the XY axes.
fn create_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = Polyline::new_closed();
    pl.add(xmin, ymin, 0.0);
    pl.add(xmax, ymin, 0.0);
    pl.add(xmax, ymax, 0.0);
    pl.add(xmin, ymax, 0.0);
    pl
}

/// Create a circle-like shape from a small polyline (approx. circle).
fn create_approx_circle(center_x: f64, center_y: f64, radius: f64) -> Polyline<f64> {
    // Two vertices with bulge 1.0 encode a full circle as two half-circle arc segments.
    let mut pl = Polyline::new_closed();
    pl.add(center_x - radius, center_y, 1.0);
    pl.add(center_x + radius, center_y, 1.0);
    pl
}

fn create_cw_rectangle(xmin: f64, ymin: f64, xmax: f64, ymax: f64) -> Polyline<f64> {
    let mut pl = create_rectangle(xmin, ymin, xmax, ymax);
    pl.invert_direction_mut();
    pl
}

fn create_shape_rects(rects: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    Shape::from_plines(
        rects
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_rectangle(xmin, ymin, xmax, ymax)),
    )
}

fn create_donut(outer: (f64, f64, f64, f64), holes: &[(f64, f64, f64, f64)]) -> Shape<f64> {
    let mut plines = vec![create_rectangle(outer.0, outer.1, outer.2, outer.3)];
    plines.extend(
        holes
            .iter()
            .map(|&(xmin, ymin, xmax, ymax)| create_cw_rectangle(xmin, ymin, xmax, ymax)),
    );
    Shape::from_plines(plines)
}

fn rotate_pline_start(pline: &Polyline<f64>, start_index: usize) -> Polyline<f64> {
    pline
        .rotate_start(start_index, pline.at(start_index).pos(), SHAPE_TEST_EPS)
        .expect("expected valid closed polyline rotation")
}

fn shape_signed_area(shape: &Shape<f64>) -> f64 {
    // Shape uses signed loop bins: CCW contributes positive area, CW contributes negative holes.
    shape
        .ccw_plines
        .iter()
        .map(|ip| ip.polyline.area())
        .chain(shape.cw_plines.iter().map(|ip| ip.polyline.area()))
        .sum()
}

fn shape_winding_number(shape: &Shape<f64>, point: Vector2<f64>) -> i32 {
    // Match Shape::boolean's non-zero winding semantics by summing every signed loop.
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

fn shape_contains(shape: &Shape<f64>, x: f64, y: f64) -> bool {
    shape_winding_number(shape, Vector2::new(x, y)) != 0
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
struct LoopSignature {
    orientation: &'static str,
    vertex_count: usize,
    area: i64,
    path_length: i64,
    extents: Option<(i64, i64, i64, i64)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
struct ShapeSignature {
    ccw_count: usize,
    cw_count: usize,
    signed_area: i64,
    perimeter_sum: i64,
    extents: Option<(i64, i64, i64, i64)>,
    loops: Vec<LoopSignature>,
}

fn sig_num(value: f64) -> i64 {
    (value * 1_000_000_000.0).round() as i64
}

fn extents_signature(
    extents: Option<static_aabb2d_index::AABB<f64>>,
) -> Option<(i64, i64, i64, i64)> {
    extents.map(|e| {
        (
            sig_num(e.min_x),
            sig_num(e.min_y),
            sig_num(e.max_x),
            sig_num(e.max_y),
        )
    })
}

fn loop_signature(pline: &Polyline<f64>) -> LoopSignature {
    LoopSignature {
        orientation: match pline.orientation() {
            PlineOrientation::Open => "open",
            PlineOrientation::CounterClockwise => "ccw",
            PlineOrientation::Clockwise => "cw",
        },
        vertex_count: pline.vertex_count(),
        area: sig_num(pline.area()),
        path_length: sig_num(pline.path_length()),
        extents: extents_signature(pline.extents()),
    }
}

fn shape_signature(shape: &Shape<f64>) -> ShapeSignature {
    let mut loops = shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| loop_signature(&ip.polyline))
        .collect::<Vec<_>>();
    loops.sort();

    ShapeSignature {
        ccw_count: shape.ccw_plines.len(),
        cw_count: shape.cw_plines.len(),
        signed_area: sig_num(shape_signed_area(shape)),
        perimeter_sum: loops.iter().map(|l| l.path_length).sum(),
        extents: extents_signature(shape.plines_index.bounds()),
        loops,
    }
}

fn assert_no_duplicate_loops(shape: &Shape<f64>) {
    let mut loops = shape_signature(shape).loops;
    loops.sort();
    for pair in loops.windows(2) {
        let signature = shape_signature(shape);
        assert_ne!(
            pair[0], pair[1],
            "shape contains duplicate coincident loops: {signature:?}"
        );
    }
}

fn assert_loop_valid(pline: &Polyline<f64>, expected_orientation: PlineOrientation) {
    assert!(pline.is_closed(), "shape loops must be closed: {pline:?}");
    assert_eq!(
        pline.orientation(),
        expected_orientation,
        "loop orientation is in the wrong shape bin: {pline:?}"
    );
    assert!(
        pline.remove_repeat_pos(SHAPE_TEST_EPS).is_none(),
        "shape loop has repeat-position vertices: {pline:?}"
    );

    for v in pline.iter_vertexes() {
        assert!(v.x.is_finite(), "non-finite x coordinate in {pline:?}");
        assert!(v.y.is_finite(), "non-finite y coordinate in {pline:?}");
        assert!(v.bulge.is_finite(), "non-finite bulge in {pline:?}");
    }
}

fn assert_shape_valid(shape: &Shape<f64>) {
    // Every boolean result should be immediately reusable as a Shape input, so verify the
    // orientation bins and spatial indexes instead of checking only area or membership.
    for ip in &shape.ccw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::CounterClockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    for ip in &shape.cw_plines {
        assert_loop_valid(&ip.polyline, PlineOrientation::Clockwise);
        let pline_extents = ip.polyline.extents().unwrap();
        let index_bounds = ip.spatial_index.bounds().unwrap();
        assert!(
            pline_extents
                .min_x
                .fuzzy_eq_eps(index_bounds.min_x, SHAPE_TEST_EPS)
                && pline_extents
                    .min_y
                    .fuzzy_eq_eps(index_bounds.min_y, SHAPE_TEST_EPS)
                && pline_extents
                    .max_x
                    .fuzzy_eq_eps(index_bounds.max_x, SHAPE_TEST_EPS)
                && pline_extents
                    .max_y
                    .fuzzy_eq_eps(index_bounds.max_y, SHAPE_TEST_EPS),
            "loop spatial index bounds do not match polyline extents"
        );
    }

    let loop_count = shape.ccw_plines.len() + shape.cw_plines.len();
    if loop_count == 0 {
        assert!(
            shape.plines_index.bounds().is_none(),
            "empty shape should not have top-level bounds"
        );
        return;
    }

    let mut expected: Option<static_aabb2d_index::AABB<f64>> = None;
    for bounds in shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| ip.spatial_index.bounds().unwrap())
    {
        expected = Some(match expected {
            None => bounds,
            Some(curr) => static_aabb2d_index::AABB::new(
                curr.min_x.min(bounds.min_x),
                curr.min_y.min(bounds.min_y),
                curr.max_x.max(bounds.max_x),
                curr.max_y.max(bounds.max_y),
            ),
        });
    }

    let expected = expected.unwrap();
    let actual = shape.plines_index.bounds().unwrap();
    assert!(
        expected.min_x.fuzzy_eq_eps(actual.min_x, SHAPE_TEST_EPS)
            && expected.min_y.fuzzy_eq_eps(actual.min_y, SHAPE_TEST_EPS)
            && expected.max_x.fuzzy_eq_eps(actual.max_x, SHAPE_TEST_EPS)
            && expected.max_y.fuzzy_eq_eps(actual.max_y, SHAPE_TEST_EPS),
        "shape top-level index bounds do not match child loop bounds"
    );

    assert_no_duplicate_loops(shape);
}

fn push_shape_probe_samples(shape: &Shape<f64>, samples: &mut Vec<(f64, f64)>) {
    let nudges = [
        (SHAPE_TEST_EPS * 1000.0, SHAPE_TEST_EPS * 1000.0),
        (-SHAPE_TEST_EPS * 1000.0, SHAPE_TEST_EPS * 1000.0),
        (SHAPE_TEST_EPS * 1000.0, -SHAPE_TEST_EPS * 1000.0),
        (-SHAPE_TEST_EPS * 1000.0, -SHAPE_TEST_EPS * 1000.0),
    ];

    for pline in shape
        .ccw_plines
        .iter()
        .chain(shape.cw_plines.iter())
        .map(|ip| &ip.polyline)
    {
        for v in pline.iter_vertexes() {
            for (dx, dy) in nudges {
                samples.push((v.x + dx, v.y + dy));
            }
        }

        for (v1, v2) in pline.iter_segments() {
            let midpoint = cavalier_contours::polyline::seg_midpoint(v1, v2);
            for (dx, dy) in nudges {
                samples.push((midpoint.x + dx, midpoint.y + dy));
            }
        }
    }
}

fn combined_extents(shapes: &[&Shape<f64>]) -> Option<static_aabb2d_index::AABB<f64>> {
    shapes
        .iter()
        .filter_map(|shape| shape.plines_index.bounds())
        .fold(None, |acc, bounds| {
            Some(match acc {
                None => bounds,
                Some(curr) => static_aabb2d_index::AABB::new(
                    curr.min_x.min(bounds.min_x),
                    curr.min_y.min(bounds.min_y),
                    curr.max_x.max(bounds.max_x),
                    curr.max_y.max(bounds.max_y),
                ),
            })
        })
}

fn semantic_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    explicit: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    let mut samples = explicit.to_vec();

    if let Some(extents) = combined_extents(&[a, b, result]) {
        let width = (extents.max_x - extents.min_x).max(1.0);
        let height = (extents.max_y - extents.min_y).max(1.0);
        // Fractions are intentionally irregular so axis-aligned grid samples are unlikely to land
        // exactly on rectangle, hole, or result boundaries, where winding-number semantics are
        // implementation-defined.
        let fractions = [0.113, 0.271, 0.389, 0.541, 0.673, 0.809, 0.947];
        for fx in fractions {
            for fy in fractions {
                samples.push((extents.min_x + width * fx, extents.min_y + height * fy));
            }
        }
        samples.push((extents.min_x - width * 0.1, extents.min_y - height * 0.1));
        samples.push((extents.max_x + width * 0.1, extents.max_y + height * 0.1));
    }

    push_shape_probe_samples(a, &mut samples);
    push_shape_probe_samples(b, &mut samples);
    samples
}

fn pline_svg_path(pline: &Polyline<f64>) -> Option<String> {
    if pline.vertex_count() == 0 {
        return None;
    }

    // Failure SVGs use path commands rather than polygons so arc-heavy regressions preserve the
    // same bulge geometry that the boolean algorithm processed.
    let first = pline.at(0);
    let first_x = first.x;
    let first_y = first.y;
    let mut path = format!("M {first_x} {first_y}");
    let segment_count = if pline.is_closed() {
        pline.vertex_count()
    } else {
        pline.vertex_count().saturating_sub(1)
    };

    for i in 0..segment_count {
        let start = pline.at(i);
        let end = pline.at((i + 1) % pline.vertex_count());
        let end_x = end.x;
        let end_y = end.y;
        if start.bulge.fuzzy_eq_eps(0.0, SHAPE_TEST_EPS) {
            path.push_str(&format!(" L {end_x} {end_y}"));
        } else {
            let (radius, _) = seg_arc_radius_and_center(start, end);
            let sweep_angle = 4.0 * start.bulge.atan();
            let large_arc = i32::from(sweep_angle.abs() > PI);
            let sweep = i32::from(start.bulge > 0.0);
            path.push_str(&format!(
                " A {radius} {radius} 0 {large_arc} {sweep} {end_x} {end_y}"
            ));
        }
    }

    if pline.is_closed() {
        path.push_str(" Z");
    }

    Some(path)
}

fn write_shape_svg(path: &Path, shapes: &[(&Shape<f64>, &'static str)]) -> std::io::Result<()> {
    let extents = combined_extents(
        &shapes
            .iter()
            .map(|(shape, _color)| *shape)
            .collect::<Vec<_>>(),
    );
    let (min_x, min_y, width, height) = if let Some(e) = extents {
        let width = (e.max_x - e.min_x).max(1.0);
        let height = (e.max_y - e.min_y).max(1.0);
        (
            e.min_x - width * 0.05,
            e.min_y - height * 0.05,
            width * 1.1,
            height * 1.1,
        )
    } else {
        (-1.0, -1.0, 2.0, 2.0)
    };

    let mut svg = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{min_x} {min_y} {width} {height}">
<g fill="none" stroke-width="0.05">
"#
    );

    for (shape, color) in shapes {
        for pline in shape
            .ccw_plines
            .iter()
            .chain(shape.cw_plines.iter())
            .map(|ip| &ip.polyline)
        {
            let Some(path) = pline_svg_path(pline) else {
                continue;
            };
            svg.push_str(&format!(
                r#"<path d="{path}" stroke="{color}" fill="{color}" fill-opacity="0.08" />
"#
            ));
        }
    }

    svg.push_str("</g>\n</svg>\n");
    fs::write(path, svg)
}

fn shape_debug_json(shape: &Shape<f64>) -> String {
    let mut result = String::new();
    result.push_str("{\n  \"ccw\": [\n");
    for (i, ip) in shape.ccw_plines.iter().enumerate() {
        if i > 0 {
            result.push_str(",\n");
        }
        result.push_str(&to_debug_json_str(&ip.polyline));
    }
    result.push_str("\n  ],\n  \"cw\": [\n");
    for (i, ip) in shape.cw_plines.iter().enumerate() {
        if i > 0 {
            result.push_str(",\n");
        }
        result.push_str(&to_debug_json_str(&ip.polyline));
    }
    result.push_str("\n  ]\n}\n");
    result
}

fn dump_shape_boolean_failure(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    sample: (f64, f64),
    expected: bool,
    actual: bool,
) {
    let Ok(dir) = env::var("CAVC_DUMP_FAILURE") else {
        return;
    };

    let dir = Path::new(&dir);
    let _ = fs::create_dir_all(dir);
    let _ = write_shape_svg(&dir.join("input_a.svg"), &[(a, "blue")]);
    let _ = write_shape_svg(&dir.join("input_b.svg"), &[(b, "red")]);
    let _ = write_shape_svg(&dir.join("result.svg"), &[(result, "green")]);
    let _ = write_shape_svg(
        &dir.join("overlay.svg"),
        &[(a, "blue"), (b, "red"), (result, "green")],
    );
    let _ = fs::write(
        dir.join("case.rs"),
        format!(
            "// op: {op:?}\n// sample: {:?}\n// expected: {expected}\n// actual: {actual}\n// a: {:?}\n// b: {:?}\n// result: {:?}\n",
            sample,
            shape_signature(a),
            shape_signature(b),
            shape_signature(result)
        ),
    );
    let _ = fs::write(dir.join("input_a.json"), shape_debug_json(a));
    let _ = fs::write(dir.join("input_b.json"), shape_debug_json(b));
    let _ = fs::write(dir.join("result.json"), shape_debug_json(result));
}

fn trace_shape_boolean_case(a: &Shape<f64>, b: &Shape<f64>, result: &Shape<f64>, op: BooleanOp) {
    if env::var_os("CAVC_TRACE_SHAPE_BOOLEAN").is_none() {
        return;
    }

    let a_signature = shape_signature(a);
    let b_signature = shape_signature(b);
    let result_signature = shape_signature(result);
    eprintln!(
        "shape boolean trace op={op:?}\na={a_signature:#?}\nb={b_signature:#?}\nresult={result_signature:#?}"
    );
}

fn assert_boolean_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    result: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    trace_shape_boolean_case(a, b, result, op);
    // Sampled membership is the semantic oracle: exact loop topology may vary, but each point
    // must obey the set operation computed from the original input shapes.
    for (x, y) in semantic_samples(a, b, result, samples) {
        let in_a = shape_contains(a, x, y);
        let in_b = shape_contains(b, x, y);
        let expected = match op {
            BooleanOp::Or => in_a || in_b,
            BooleanOp::And => in_a && in_b,
            BooleanOp::Not => in_a && !in_b,
            BooleanOp::Xor => in_a != in_b,
        };
        let actual = shape_contains(result, x, y);
        if actual != expected {
            dump_shape_boolean_failure(a, b, result, op, (x, y), expected, actual);
        }
        let result_area = shape_signed_area(result);
        assert_eq!(
            actual, expected,
            "sample ({x}, {y}) mismatch for {op:?}: in_a={in_a}, in_b={in_b}, result={actual}, result_area={result_area}"
        );
    }
}

fn assert_boolean_result(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    expected_ccw_count: usize,
    expected_cw_count: usize,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    trace_shape_boolean_case(a, b, &result, op);
    assert_shape_valid(&result);
    let result_area = shape_signed_area(&result);
    assert_eq!(
        result.ccw_plines.len(),
        expected_ccw_count,
        "unexpected ccw loop count for {op:?}"
    );
    assert_eq!(
        result.cw_plines.len(),
        expected_cw_count,
        "unexpected cw loop count for {op:?}"
    );
    assert!(
        result_area.fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {result_area}"
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

fn assert_boolean_area_and_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    expected_area: f64,
    samples: &[(f64, f64)],
) {
    let result = a.boolean(b, op);
    trace_shape_boolean_case(a, b, &result, op);
    assert_shape_valid(&result);
    let result_area = shape_signed_area(&result);
    assert!(
        result_area.fuzzy_eq_eps(expected_area, 1e-5),
        "unexpected signed area for {op:?}: expected {expected_area}, got {result_area}"
    );
    assert_boolean_samples(a, b, &result, op, samples);
}

fn assert_commutative_samples(
    a: &Shape<f64>,
    b: &Shape<f64>,
    op: BooleanOp,
    samples: &[(f64, f64)],
) {
    let ab = a.boolean(b, op);
    let ba = b.boolean(a, op);

    assert_shape_valid(&ab);
    assert_shape_valid(&ba);
    let ab_area = shape_signed_area(&ab);
    let ba_area = shape_signed_area(&ba);
    assert!(
        ab_area.fuzzy_eq_eps(ba_area, 1e-5),
        "commutative area mismatch for {op:?}: ab={ab_area}, ba={ba_area}"
    );
    assert_eq!(
        ab.ccw_plines.len(),
        ba.ccw_plines.len(),
        "commutative ccw loop count mismatch for {op:?}"
    );
    assert_eq!(
        ab.cw_plines.len(),
        ba.cw_plines.len(),
        "commutative cw loop count mismatch for {op:?}"
    );
    assert_boolean_samples(a, b, &ab, op, samples);
    assert_boolean_samples(b, a, &ba, op, samples);
}

fn assert_shapes_equivalent_by_samples(a: &Shape<f64>, b: &Shape<f64>, samples: &[(f64, f64)]) {
    assert_shape_valid(a);
    assert_shape_valid(b);
    let a_area = shape_signed_area(a);
    let b_area = shape_signed_area(b);
    assert!(
        a_area.fuzzy_eq_eps(b_area, 1e-5),
        "shape area mismatch: left={a_area}, right={b_area}"
    );
    assert_eq!(
        a.ccw_plines.len(),
        b.ccw_plines.len(),
        "ccw loop count mismatch"
    );
    assert_eq!(
        a.cw_plines.len(),
        b.cw_plines.len(),
        "cw loop count mismatch"
    );

    for &(x, y) in samples {
        assert_eq!(
            shape_contains(a, x, y),
            shape_contains(b, x, y),
            "sample ({x}, {y}) equivalence mismatch"
        );
    }
}

#[test]
fn shape_boolean_empty_identities_are_valid() {
    // Empty-shape identities catch missing unused-loop retention and accidental top-level bounds.
    let empty = Shape::<f64>::empty();
    let solid = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(-1.0, -1.0), (1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_shape_valid(&empty);
    assert_shape_valid(&solid);

    assert_boolean_result(&solid, &empty, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&empty, &solid, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&solid, &empty, BooleanOp::Xor, 100.0, 1, 0, &samples);
}

#[test]
fn shape_boolean_rectangle_containment_all_ops() {
    let outer = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let inner = Shape::from_plines(vec![create_rectangle(2.0, 2.0, 4.0, 4.0)]);
    let samples = [(1.0, 1.0), (3.0, 3.0), (8.0, 8.0), (12.0, 12.0)];

    assert_boolean_result(&outer, &inner, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::And, 4.0, 1, 0, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Not, 96.0, 1, 1, &samples);
    assert_boolean_result(&outer, &inner, BooleanOp::Xor, 96.0, 1, 1, &samples);
}

#[test]
fn shape_boolean_edge_touching_rectangles_do_not_create_area() {
    let left = Shape::from_plines(vec![create_rectangle(0.0, 0.0, 10.0, 10.0)]);
    let right = Shape::from_plines(vec![create_rectangle(10.0, 0.0, 20.0, 10.0)]);
    let samples = [(5.0, 5.0), (15.0, 5.0), (10.5, 5.0), (-1.0, 5.0)];

    assert_boolean_result(&left, &right, BooleanOp::Or, 200.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Not, 100.0, 1, 0, &samples);
    assert_boolean_result(&left, &right, BooleanOp::Xor, 200.0, 1, 0, &samples);
}

#[test]
fn shape_boolean_square_inside_hole_respects_empty_space() {
    // A positive island inside a hole is disjoint from the donut's filled area but must be
    // preserved by union and XOR as a nested positive loop.
    let donut = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_cw_rectangle(3.0, 3.0, 7.0, 7.0),
    ]);
    let island = Shape::from_plines(vec![create_rectangle(4.0, 4.0, 6.0, 6.0)]);
    let samples = [(1.0, 1.0), (3.5, 3.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(&donut, &island, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&island, &donut, BooleanOp::Or, 88.0, 2, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Not, 84.0, 1, 1, &samples);
    assert_boolean_result(&donut, &island, BooleanOp::Xor, 88.0, 2, 1, &samples);
}

#[test]
fn shape_boolean_identical_rectangle_all_ops() {
    let rect = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let samples = [(1.0, 1.0), (5.0, 5.0), (11.0, 11.0)];

    assert_boolean_result(&rect, &rect, BooleanOp::Or, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::And, 100.0, 1, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&rect, &rect, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_identical_donut_all_ops() {
    // Equal shapes with holes are an identity case for every boolean operator. This specifically
    // protects against treating matching CW holes as positive material during shape-level union.
    let donut = create_donut((0.0, 0.0, 12.0, 12.0), &[(2.0, 2.0, 10.0, 10.0)]);
    let same_donut = donut.clone();
    let samples = [(1.0, 1.0), (6.0, 6.0), (11.0, 11.0), (13.0, 13.0)];

    assert_boolean_result(&donut, &same_donut, BooleanOp::Or, 80.0, 1, 1, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::And, 80.0, 1, 1, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut, &same_donut, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_same_shape_identity_ignores_loop_order_and_start_index() {
    // The identity fast path must be geometric, not storage-order based. This variant keeps the
    // same signed loop bins but reorders the CCW loops and rotates every closed-loop start index.
    let base = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 20.0, 20.0),
        create_cw_rectangle(5.0, 5.0, 15.0, 15.0),
        create_rectangle(8.0, 8.0, 12.0, 12.0),
    ]);
    let variant = Shape::from_plines(vec![
        rotate_pline_start(&create_rectangle(8.0, 8.0, 12.0, 12.0), 2),
        rotate_pline_start(&create_cw_rectangle(5.0, 5.0, 15.0, 15.0), 1),
        rotate_pline_start(&create_rectangle(0.0, 0.0, 20.0, 20.0), 3),
    ]);
    let samples = [
        (2.0, 2.0),
        (6.0, 6.0),
        (10.0, 10.0),
        (18.0, 18.0),
        (25.0, 25.0),
    ];

    assert_boolean_result(&base, &variant, BooleanOp::Or, 316.0, 2, 1, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::And, 316.0, 2, 1, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::Not, 0.0, 0, 0, &samples);
    assert_boolean_result(&base, &variant, BooleanOp::Xor, 0.0, 0, 0, &samples);
}

#[test]
fn shape_boolean_commutative_overlapping_rectangles() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let b = create_shape_rects(&[(5.0, 5.0, 15.0, 15.0)]);
    let samples = [(2.0, 2.0), (7.5, 7.5), (12.0, 12.0), (16.0, 16.0)];

    assert_commutative_samples(&a, &b, BooleanOp::Or, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::And, &samples);
    assert_commutative_samples(&a, &b, BooleanOp::Xor, &samples);
}

#[test]
fn shape_boolean_multi_island_disjoint_identities() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let b = create_shape_rects(&[(100.0, 0.0, 110.0, 10.0)]);
    let samples = [(5.0, 5.0), (25.0, 5.0), (105.0, 5.0), (50.0, 5.0)];

    assert_boolean_result(&a, &b, BooleanOp::Or, 300.0, 3, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Not, 200.0, 2, 0, &samples);
    assert_boolean_result(&a, &b, BooleanOp::Xor, 300.0, 3, 0, &samples);
}

#[test]
fn shape_boolean_bridge_overlaps_two_islands() {
    // One input loop intersects two separate islands, forcing pairwise results to merge into a
    // single output component for union while preserving separated pieces for intersection.
    let islands = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0), (20.0, 0.0, 30.0, 10.0)]);
    let bridge = create_shape_rects(&[(5.0, -5.0, 25.0, 15.0)]);
    let samples = [
        (2.5, 5.0),
        (7.5, 5.0),
        (15.0, 0.0),
        (22.5, 5.0),
        (27.5, 5.0),
        (15.0, 12.5),
        (15.0, 20.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 500.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 100.0, 2, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 100.0, 2, 0, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::Or, &samples);
    assert_commutative_samples(&islands, &bridge, BooleanOp::And, &samples);
}

#[test]
fn shape_boolean_ring_difference_adds_second_hole() {
    // Difference against material inside the filled ring should add a new hole without disturbing
    // an existing unrelated hole.
    let ring = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let cutter_in_filled_ring = create_shape_rects(&[(1.0, 1.0, 2.0, 2.0)]);
    let samples = [(1.5, 1.5), (2.5, 2.5), (5.0, 5.0), (8.0, 8.0), (11.0, 11.0)];

    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Not,
        83.0,
        1,
        2,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::And,
        1.0,
        1,
        0,
        &samples,
    );
    assert_boolean_result(
        &ring,
        &cutter_in_filled_ring,
        BooleanOp::Xor,
        83.0,
        1,
        2,
        &samples,
    );
}

#[test]
fn shape_boolean_loop_order_and_start_index_are_irrelevant() {
    // Shape booleans should depend on geometry, not storage order or the chosen start vertex of a
    // closed loop. This protects the pairwise bookkeeping from index-order assumptions.
    let base_a = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_rectangle(20.0, 0.0, 30.0, 10.0),
    ]);
    let variant_a = Shape::from_plines(vec![
        rotate_pline_start(&create_rectangle(20.0, 0.0, 30.0, 10.0), 2),
        rotate_pline_start(&create_rectangle(0.0, 0.0, 10.0, 10.0), 3),
    ]);

    let base_b = Shape::from_plines(vec![create_rectangle(5.0, -5.0, 25.0, 15.0)]);
    let variant_b = Shape::from_plines(vec![rotate_pline_start(
        &create_rectangle(5.0, -5.0, 25.0, 15.0),
        1,
    )]);

    let samples = [
        (2.0, 2.0),
        (5.5, 5.5),
        (8.0, 8.0),
        (15.0, 1.0),
        (15.0, 4.0),
        (22.0, 5.0),
        (28.0, 5.0),
        (40.0, 40.0),
    ];

    for op in [
        BooleanOp::Or,
        BooleanOp::And,
        BooleanOp::Not,
        BooleanOp::Xor,
    ] {
        let base_result = base_a.boolean(&base_b, op);
        let variant_result = variant_a.boolean(&variant_b, op);
        assert_shapes_equivalent_by_samples(&base_result, &variant_result, &samples);
        assert_boolean_samples(&base_a, &base_b, &base_result, op, &samples);
        assert_boolean_samples(&variant_a, &variant_b, &variant_result, op, &samples);
    }
}

#[test]
fn shape_boolean_chain_bridge_merges_three_islands() {
    // The bridge overlaps three islands transitively; a one-pass pairwise collector can easily
    // leave duplicate or partially merged loops here if final assembly is not robust.
    let islands = create_shape_rects(&[
        (0.0, 0.0, 6.0, 6.0),
        (10.0, 0.0, 16.0, 6.0),
        (20.0, 0.0, 26.0, 6.0),
    ]);
    let bridge = create_shape_rects(&[(3.0, -2.0, 23.0, 8.0)]);
    let samples = [
        (1.0, 1.0),
        (5.0, 1.0),
        (8.0, 0.0),
        (13.0, 3.0),
        (18.0, 0.0),
        (24.0, 3.0),
        (12.0, 7.0),
        (30.0, 3.0),
    ];

    assert_boolean_result(&islands, &bridge, BooleanOp::Or, 236.0, 1, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::And, 72.0, 3, 0, &samples);
    assert_boolean_result(&islands, &bridge, BooleanOp::Not, 36.0, 2, 0, &samples);
    assert_boolean_area_and_samples(&islands, &bridge, BooleanOp::Xor, 164.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_splits_ring_without_filling_hole() {
    // Regression: a cutter that crosses a hole boundary must not cause the original hole to be
    // treated as filled material during intersection or difference assembly.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_result(&ring, &cutter, BooleanOp::And, 24.0, 1, 0, &samples);
    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Not, 104.0, &samples);
}

#[test]
fn shape_boolean_hole_boundary_cutter_xor() {
    // Regression for XOR's (A - B) union (B - A) composition: the B - A island lies inside the
    // merged hole from A - B, so union must keep both nested loops instead of filling the hole.
    let ring = create_donut((0.0, 0.0, 12.0, 12.0), &[(4.0, 4.0, 8.0, 8.0)]);
    let cutter = create_shape_rects(&[(6.0, 2.0, 10.0, 10.0)]);
    let samples = [
        (2.0, 2.0),
        (5.0, 5.0),
        (7.0, 3.0),
        (7.0, 7.0),
        (9.0, 9.0),
        (11.0, 11.0),
        (13.0, 13.0),
    ];

    assert_boolean_area_and_samples(&ring, &cutter, BooleanOp::Xor, 112.0, &samples);
}

#[test]
fn shape_boolean_point_touching_rectangles_do_not_create_intersection_area() {
    let lower_left = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let upper_right = create_shape_rects(&[(10.0, 10.0, 20.0, 20.0)]);
    let samples = [(5.0, 5.0), (15.0, 15.0), (10.001, 10.001), (9.999, 9.999)];

    assert_boolean_area_and_samples(&lower_left, &upper_right, BooleanOp::Or, 200.0, &samples);
    assert_boolean_result(
        &lower_left,
        &upper_right,
        BooleanOp::And,
        0.0,
        0,
        0,
        &samples,
    );
    assert_boolean_area_and_samples(&lower_left, &upper_right, BooleanOp::Xor, 200.0, &samples);
}

#[test]
fn shape_boolean_partially_shared_edge_is_regularized() {
    let bottom = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let top = create_shape_rects(&[(5.0, 10.0, 15.0, 20.0)]);
    let samples = [(2.0, 2.0), (7.5, 9.999), (7.5, 10.001), (12.0, 12.0)];

    assert_boolean_area_and_samples(&bottom, &top, BooleanOp::Or, 200.0, &samples);
    assert_boolean_result(&bottom, &top, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_area_and_samples(&bottom, &top, BooleanOp::Not, 100.0, &samples);
}

#[test]
fn shape_boolean_thin_sliver_overlap_keeps_area_identities() {
    let a = create_shape_rects(&[(0.0, 0.0, 10.0, 10.0)]);
    let b = create_shape_rects(&[(9.999, 0.0, 20.0, 10.0)]);
    let samples = [(5.0, 5.0), (9.9995, 5.0), (10.0005, 5.0), (15.0, 5.0)];

    assert_boolean_area_and_samples(&a, &b, BooleanOp::Or, 200.0, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::And, 0.01, &samples);
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Not, 99.99, &samples);
    // The current lower-level boolean regularizes this tiny overlap on the XOR path; keep the
    // semantic samples active while accepting the current stable area.
    assert_boolean_area_and_samples(&a, &b, BooleanOp::Xor, 199.99, &samples);
}

#[test]
fn shape_boolean_circle_disjoint_tangent_and_identical_cases() {
    let c1_pline = create_approx_circle(0.0, 0.0, 2.0);
    let c2_pline = create_approx_circle(6.0, 0.0, 2.0);
    let tangent_pline = create_approx_circle(4.0, 0.0, 2.0);
    let c1_area = c1_pline.area();

    let c1 = Shape::from_plines(vec![c1_pline.clone()]);
    let c2 = Shape::from_plines(vec![c2_pline]);
    let tangent = Shape::from_plines(vec![tangent_pline]);
    let c1_again = Shape::from_plines(vec![c1_pline]);

    let disjoint_samples = [(0.0, 0.0), (6.0, 0.0), (3.0, 0.0), (10.0, 0.0)];
    assert_boolean_area_and_samples(&c1, &c2, BooleanOp::Or, c1_area * 2.0, &disjoint_samples);
    assert_boolean_result(&c1, &c2, BooleanOp::And, 0.0, 0, 0, &disjoint_samples);
    assert_boolean_area_and_samples(&c1, &c2, BooleanOp::Xor, c1_area * 2.0, &disjoint_samples);

    let tangent_samples = [(0.0, 0.0), (4.0, 0.0), (2.0001, 0.0), (8.0, 0.0)];
    assert_boolean_area_and_samples(
        &c1,
        &tangent,
        BooleanOp::Or,
        c1_area * 2.0,
        &tangent_samples,
    );
    assert_boolean_result(&c1, &tangent, BooleanOp::And, 0.0, 0, 0, &tangent_samples);

    let identical_samples = [(0.0, 0.0), (1.0, 0.0), (2.1, 0.0), (-2.1, 0.0)];
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::Or,
        c1_area,
        1,
        0,
        &identical_samples,
    );
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::And,
        c1_area,
        1,
        0,
        &identical_samples,
    );
    assert_boolean_result(
        &c1,
        &c1_again,
        BooleanOp::Not,
        0.0,
        0,
        0,
        &identical_samples,
    );
}

#[test]
fn shape_boolean_donut_square_outside_and_hole_boundary_cases() {
    let donut = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let outside_square = create_shape_rects(&[(20.0, 20.0, 22.0, 22.0)]);

    let outside_samples = [(1.0, 1.0), (5.0, 5.0), (21.0, 21.0), (15.0, 15.0)];
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::Or,
        88.0,
        2,
        1,
        &outside_samples,
    );
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::And,
        0.0,
        0,
        0,
        &outside_samples,
    );
    assert_boolean_result(
        &donut,
        &outside_square,
        BooleanOp::Not,
        84.0,
        1,
        1,
        &outside_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &outside_square,
        BooleanOp::Xor,
        88.0,
        &outside_samples,
    );
}

#[test]
#[ignore = "documents current hole-boundary union area inflation"]
fn reported_shape_boolean_hole_boundary_union_area_inflation_1() {
    let donut = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let hole_boundary_square = create_shape_rects(&[(2.0, 2.0, 5.0, 5.0)]);

    let boundary_samples = [(2.5, 2.5), (4.5, 4.5), (1.0, 1.0), (8.0, 8.0), (11.0, 11.0)];
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Or,
        88.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::And,
        5.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Not,
        79.0,
        &boundary_samples,
    );
    assert_boolean_area_and_samples(
        &donut,
        &hole_boundary_square,
        BooleanOp::Xor,
        83.0,
        &boundary_samples,
    );
}

#[test]
fn shape_boolean_two_disjoint_donuts_and_nested_ring() {
    let donut_a = create_donut((0.0, 0.0, 10.0, 10.0), &[(3.0, 3.0, 7.0, 7.0)]);
    let donut_b = create_donut((20.0, 0.0, 30.0, 10.0), &[(23.0, 3.0, 27.0, 7.0)]);
    let samples = [
        (1.0, 1.0),
        (5.0, 5.0),
        (21.0, 1.0),
        (25.0, 5.0),
        (15.0, 5.0),
    ];

    assert_boolean_result(&donut_a, &donut_b, BooleanOp::Or, 168.0, 2, 2, &samples);
    assert_boolean_result(&donut_a, &donut_b, BooleanOp::And, 0.0, 0, 0, &samples);
    assert_boolean_result(&donut_a, &donut_b, BooleanOp::Not, 84.0, 1, 1, &samples);
    assert_boolean_area_and_samples(&donut_a, &donut_b, BooleanOp::Xor, 168.0, &samples);

    let nested = Shape::from_plines(vec![
        create_rectangle(0.0, 0.0, 10.0, 10.0),
        create_cw_rectangle(2.0, 2.0, 8.0, 8.0),
        create_rectangle(4.0, 4.0, 6.0, 6.0),
    ]);
    assert_shape_valid(&nested);
    assert!(shape_signed_area(&nested).fuzzy_eq_eps(68.0, 1e-5));
    assert_boolean_result(
        &nested,
        &Shape::<f64>::empty(),
        BooleanOp::Or,
        68.0,
        2,
        1,
        &[(1.0, 1.0), (3.0, 3.0), (5.0, 5.0), (11.0, 11.0)],
    );
}

#[cfg(test)]
mod shape_boolean_proptests {
    use super::*;
    use proptest::prelude::*;

    type Rect = (f64, f64, f64, f64);
    type RectPair = (Rect, Rect);

    fn rect_strategy() -> impl Strategy<Value = Rect> {
        // Integer coordinates keep expected areas exact while still exploring many placements.
        (-100i32..100, -100i32..100, 1i32..40, 1i32..40).prop_map(|(x, y, w, h)| {
            let xmin = f64::from(x);
            let ymin = f64::from(y);
            (xmin, ymin, xmin + f64::from(w), ymin + f64::from(h))
        })
    }

    fn rect_area(rect: Rect) -> f64 {
        (rect.2 - rect.0) * (rect.3 - rect.1)
    }

    fn rect_center(rect: Rect) -> (f64, f64) {
        ((rect.0 + rect.2) * 0.5, (rect.1 + rect.3) * 0.5)
    }

    fn outside_sample(rect: Rect) -> (f64, f64) {
        (rect.2 + 10.0, rect.3 + 10.0)
    }

    fn rect_overlap_area(a: Rect, b: Rect) -> f64 {
        let xmin = a.0.max(b.0);
        let ymin = a.1.max(b.1);
        let xmax = a.2.min(b.2);
        let ymax = a.3.min(b.3);
        if xmax <= xmin || ymax <= ymin {
            0.0
        } else {
            (xmax - xmin) * (ymax - ymin)
        }
    }

    fn overlapping_rect_pair_strategy() -> impl Strategy<Value = RectPair> {
        // Bias toward guaranteed overlaps so area identities exercise clipping instead of only
        // unused-loop retention.
        (-50i32..50, -50i32..50, 6i32..40, 6i32..40, 1i32..6, 1i32..6).prop_map(
            |(x, y, w, h, inset_x, inset_y)| {
                let a = (
                    f64::from(x),
                    f64::from(y),
                    f64::from(x + w),
                    f64::from(y + h),
                );
                let b = (
                    f64::from(x + inset_x),
                    f64::from(y + inset_y),
                    f64::from(x + w + inset_x),
                    f64::from(y + h + inset_y),
                );
                (a, b)
            },
        )
    }

    fn donut_rect_strategy() -> impl Strategy<Value = RectPair> {
        (-50i32..50, -50i32..50, 16i32..60, 16i32..60).prop_map(|(x, y, w, h)| {
            let margin_x = (w / 4).max(3);
            let margin_y = (h / 4).max(3);
            let outer = (
                f64::from(x),
                f64::from(y),
                f64::from(x + w),
                f64::from(y + h),
            );
            let inner = (
                f64::from(x + margin_x),
                f64::from(y + margin_y),
                f64::from(x + w - margin_x),
                f64::from(y + h - margin_y),
            );
            (outer, inner)
        })
    }

    fn translate_rect(rect: Rect, dx: f64, dy: f64) -> Rect {
        (rect.0 + dx, rect.1 + dy, rect.2 + dx, rect.3 + dy)
    }

    proptest! {
        #![proptest_config(ProptestConfig {
            cases: 64,
            failure_persistence: None,
            ..ProptestConfig::default()
        })]

        #[test]
        fn rectangle_empty_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let empty = Shape::<f64>::empty();
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &empty, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Not, area, 1, 0, &samples);
            assert_boolean_result(&empty, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &empty, BooleanOp::Xor, area, 1, 0, &samples);
        }

        #[test]
        fn translated_disjoint_rectangles_obey_basic_area_laws(
            rect in rect_strategy(),
            gap in 1i32..50,
            w2 in 1i32..40,
            h2 in 1i32..40,
        ) {
            let b = (
                rect.2 + f64::from(gap),
                rect.1,
                rect.2 + f64::from(gap + w2),
                rect.1 + f64::from(h2),
            );
            let a_shape = create_shape_rects(&[rect]);
            let b_shape = create_shape_rects(&[b]);
            let samples = [rect_center(rect), rect_center(b), outside_sample(b)];

            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Or,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Not, rect_area(rect), 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Xor,
                rect_area(rect) + rect_area(b),
                2,
                0,
                &samples,
            );
        }

        #[test]
        fn rectangle_self_identities_hold(rect in rect_strategy()) {
            let shape = create_shape_rects(&[rect]);
            let area = rect_area(rect);
            let samples = [rect_center(rect), outside_sample(rect)];

            assert_boolean_result(&shape, &shape, BooleanOp::Or, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::And, area, 1, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&shape, &shape, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn overlapping_rectangles_obey_area_identities((a, b) in overlapping_rect_pair_strategy()) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let overlap = rect_overlap_area(a, b);
            let union_area = rect_area(a) + rect_area(b) - overlap;
            let xor_area = rect_area(a) + rect_area(b) - 2.0 * overlap;
            let samples = [
                ((a.0 + b.0) * 0.5, (a.1 + b.1) * 0.5),
                ((a.2 + b.2) * 0.5, (a.3 + b.3) * 0.5),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
                outside_sample(b),
            ];

            assert_boolean_result(&a_shape, &b_shape, BooleanOp::Or, union_area, 1, 0, &samples);
            assert_boolean_result(&a_shape, &b_shape, BooleanOp::And, overlap, 1, 0, &samples);
            assert_boolean_result(
                &a_shape,
                &b_shape,
                BooleanOp::Not,
                rect_area(a) - overlap,
                1,
                0,
                &samples,
            );
            assert_boolean_area_and_samples(&a_shape, &b_shape, BooleanOp::Xor, xor_area, &samples);
        }

        #[test]
        fn donut_empty_and_self_identities_hold((outer, inner) in donut_rect_strategy()) {
            let donut = create_donut(outer, &[inner]);
            let same_donut = donut.clone();
            let empty = Shape::<f64>::empty();
            let area = rect_area(outer) - rect_area(inner);
            let samples = [rect_center(outer), rect_center(inner), outside_sample(outer)];

            assert_boolean_area_and_samples(&donut, &empty, BooleanOp::Or, area, &samples);
            assert_boolean_result(&donut, &empty, BooleanOp::And, 0.0, 0, 0, &samples);
            assert_boolean_area_and_samples(&donut, &empty, BooleanOp::Not, area, &samples);
            assert_boolean_area_and_samples(&donut, &same_donut, BooleanOp::Or, area, &samples);
            assert_boolean_area_and_samples(&donut, &same_donut, BooleanOp::And, area, &samples);
            assert_boolean_result(&donut, &same_donut, BooleanOp::Not, 0.0, 0, 0, &samples);
            assert_boolean_result(&donut, &same_donut, BooleanOp::Xor, 0.0, 0, 0, &samples);
        }

        #[test]
        fn rectangle_difference_decomposes_into_difference_and_intersection(
            (a, b) in overlapping_rect_pair_strategy(),
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let diff = a_shape.boolean(&b_shape, BooleanOp::Not);
            let intersection = a_shape.boolean(&b_shape, BooleanOp::And);
            let recomposed = diff.boolean(&intersection, BooleanOp::Or);
            let samples = [
                rect_center(a),
                rect_center(b),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
                outside_sample(b),
            ];

            assert_shapes_equivalent_by_samples(&a_shape, &recomposed, &samples);
        }

        #[test]
        fn translated_overlapping_rectangles_preserve_boolean_results(
            (a, b) in overlapping_rect_pair_strategy(),
            dx in -200i32..200,
            dy in -200i32..200,
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let dx = f64::from(dx);
            let dy = f64::from(dy);
            let translated_a = create_shape_rects(&[translate_rect(a, dx, dy)]);
            let translated_b = create_shape_rects(&[translate_rect(b, dx, dy)]);
            let samples = [
                rect_center(a),
                rect_center(b),
                outside_sample(b),
                ((a.0.max(b.0) + a.2.min(b.2)) * 0.5, (a.1.max(b.1) + a.3.min(b.3)) * 0.5),
            ];

            for op in [BooleanOp::Or, BooleanOp::And, BooleanOp::Not, BooleanOp::Xor] {
                let base = a_shape.boolean(&b_shape, op);
                let translated = translated_a.boolean(&translated_b, op);
                assert_shape_valid(&base);
                assert_shape_valid(&translated);
                assert!(
                    shape_signed_area(&base).fuzzy_eq_eps(shape_signed_area(&translated), 1e-5),
                    "translation changed area for {op:?}"
                );

                for (x, y) in samples {
                    assert_eq!(
                        shape_contains(&base, x, y),
                        shape_contains(&translated, x + dx, y + dy),
                        "translation changed sampled membership for {op:?}"
                    );
                }
            }
        }

        #[test]
        fn scaled_overlapping_rectangles_scale_result_area(
            (a, b) in overlapping_rect_pair_strategy(),
            scale in 2i32..8,
        ) {
            let a_shape = create_shape_rects(&[a]);
            let b_shape = create_shape_rects(&[b]);
            let scale = f64::from(scale);
            let mut scaled_a = a_shape.clone();
            let mut scaled_b = b_shape.clone();
            scaled_a.scale_mut(scale);
            scaled_b.scale_mut(scale);

            for op in [BooleanOp::Or, BooleanOp::And, BooleanOp::Not, BooleanOp::Xor] {
                let base = a_shape.boolean(&b_shape, op);
                let scaled = scaled_a.boolean(&scaled_b, op);
                assert_shape_valid(&base);
                assert_shape_valid(&scaled);
                assert!(
                    (shape_signed_area(&base) * scale * scale)
                        .fuzzy_eq_eps(shape_signed_area(&scaled), 1e-4),
                    "uniform scaling changed area incorrectly for {op:?}"
                );
            }
        }
    }
}

#[test]
fn shape_empty_union_returns_self() {
    let shape_empty = Shape::<f64>::empty();
    let shape_nonempty = {
        let mut p = Polyline::new_closed();
        p.add(0.0, 0.0, 0.0);
        p.add(10.0, 0.0, 0.0);
        p.add(10.0, 10.0, 0.0);
        p.add(0.0, 10.0, 0.0);
        Shape::from_plines(vec![p])
    };
    let union_result = shape_empty.union(&shape_nonempty);
    // Expect same shape as `shape_nonempty`:
    assert_eq!(
        union_result.ccw_plines.len(),
        shape_nonempty.ccw_plines.len()
    );
    assert_eq!(union_result.cw_plines.len(), shape_nonempty.cw_plines.len());
}

#[test]
fn shape_disjoint_union_keeps_both() {
    // Two squares far apart, union => 2 loops in resulting shape
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let union_result = shape1.union(&shape2);
    // Should have 2 separate loops, both ccw:
    assert_eq!(union_result.ccw_plines.len(), 2);
    assert!(union_result.cw_plines.is_empty());
}

#[test]
fn shape_contains_shape_union_gives_first() {
    // Big square fully containing a smaller square => union is just bigger square
    let big_sq = pline_closed![
        (0.0, 0.0, 0.0),
        (100.0, 0.0, 0.0),
        (100.0, 100.0, 0.0),
        (0.0, 100.0, 0.0)
    ];
    let small_sq = pline_closed![
        (10.0, 10.0, 0.0),
        (20.0, 10.0, 0.0),
        (20.0, 20.0, 0.0),
        (10.0, 20.0, 0.0)
    ];
    let shape_big = Shape::from_plines(vec![big_sq.clone()]);
    let shape_small = Shape::from_plines(vec![small_sq]);

    let union_result = shape_big.union(&shape_small);
    // Should have same bounding box and area as `big_sq`
    assert_eq!(union_result.ccw_plines.len(), 1);
    let union_pline = &union_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(union_pline.area(), big_sq.area());
}

#[test]
fn shape_difference_completely_disjoint_no_intersection() {
    // Two squares far apart => difference => same as first
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1.clone()]);
    let shape2 = Shape::from_plines(vec![square2]);
    let diff_result = shape1.difference(&shape2);
    assert_eq!(diff_result.ccw_plines.len(), 1);
    assert!(diff_result.cw_plines.is_empty());

    let out_pline = &diff_result.ccw_plines[0].polyline;
    assert_fuzzy_eq!(out_pline.area(), square1.area());
}

#[test]
fn shape_intersection_disjoint_is_empty() {
    // Two squares far apart => intersection => empty
    let square1 = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let square2 = pline_closed![
        (100.0, 100.0, 0.0),
        (110.0, 100.0, 0.0),
        (110.0, 110.0, 0.0),
        (100.0, 110.0, 0.0)
    ];
    let shape1 = Shape::from_plines(vec![square1]);
    let shape2 = Shape::from_plines(vec![square2]);

    let intersect_result = shape1.intersection(&shape2);
    // Should be empty
    assert!(intersect_result.ccw_plines.is_empty());
    assert!(intersect_result.cw_plines.is_empty());
}

#[test]
fn shape_open_pline_included_in_boolean() {
    // If shape has open polyline, those won't be closed => won't form loops
    // but let's just confirm no panic:
    let open_pl = pline_open![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![open_pl]);
    let shape_empty = Shape::<f64>::empty();
    let union_result = shape.union(&shape_empty);
    // union => same as shape
    // though the shape is "not fully closed," area = 0, etc.
    assert_eq!(union_result.ccw_plines.len(), shape.ccw_plines.len());
}

#[test]
fn shape_parallel_offset_outward() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(-1.0, Default::default());
    // Expect single loop with bigger bounding box
    assert_eq!(offset_result.ccw_plines.len(), 1);
    let out_pline = &offset_result.ccw_plines[0].polyline;
    assert!(out_pline.extents().unwrap().max_x > 4.9);
    assert!(out_pline.extents().unwrap().max_y > 2.9);
}

#[test]
fn shape_parallel_offset_inward_collapses() {
    // Inward offset that collapses rectangle
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (4.0, 0.0, 0.0),
        (4.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let shape = Shape::from_plines(vec![rect]);
    let offset_result = shape.parallel_offset(2.0, Default::default());
    // Collapsed => empty
    assert!(offset_result.ccw_plines.is_empty());
    assert!(offset_result.cw_plines.is_empty());
}

#[test]
fn shape_translate_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.translate_mut(5.0, 5.0);
    // bounding box now from (5,5) to (15,15)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 5.0);
    assert_fuzzy_eq!(aabb.min_y, 5.0);
    assert_fuzzy_eq!(aabb.max_x, 15.0);
    assert_fuzzy_eq!(aabb.max_y, 15.0);
}

#[test]
fn shape_scale_works() {
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.scale_mut(2.0);
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, 0.0);
    assert_fuzzy_eq!(aabb.max_x, 20.0);
    assert_fuzzy_eq!(aabb.max_y, 20.0);
}

#[test]
fn shape_mirror_x_works() {
    // Mirror about X (meaning y -> -y)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (2.0, 0.0, 0.0),
        (2.0, 2.0, 0.0),
        (0.0, 2.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.mirror_x_mut();
    // bounding box now from (0, -2) to (2, 0)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 0.0);
    assert_fuzzy_eq!(aabb.min_y, -2.0);
    assert_fuzzy_eq!(aabb.max_x, 2.0);
    assert_fuzzy_eq!(aabb.max_y, 0.0);
}

#[test]
fn shape_center_works() {
    // shape is from (0,0) to (10,10)
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (10.0, 0.0, 0.0),
        (10.0, 10.0, 0.0),
        (0.0, 10.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    shape.center_mut();
    let aabb = shape.plines_index.bounds().unwrap();
    // bounding box is now from (-5,-5) to (5,5)
    assert_fuzzy_eq!(aabb.min_x, -5.0);
    assert_fuzzy_eq!(aabb.min_y, -5.0);
    assert_fuzzy_eq!(aabb.max_x, 5.0);
    assert_fuzzy_eq!(aabb.max_y, 5.0);
}

#[test]
fn shape_transform_mut() {
    // Apply a 2D transform [a b; c d] plus translation.
    let rect = pline_closed![
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        (1.0, 1.0, 0.0),
        (0.0, 1.0, 0.0)
    ];
    let mut shape = Shape::from_plines(vec![rect]);
    // Matrix [2 0; 0 2] plus (10,10) translation.
    shape.transform_mut(2.0, 0.0, 0.0, 2.0, 10.0, 10.0);

    // bounding box now from (10,10) to (12,12)
    let aabb = shape.plines_index.bounds().unwrap();
    assert_fuzzy_eq!(aabb.min_x, 10.0);
    assert_fuzzy_eq!(aabb.min_y, 10.0);
    assert_fuzzy_eq!(aabb.max_x, 12.0);
    assert_fuzzy_eq!(aabb.max_y, 12.0);
}

#[test]
fn transform_builder_scale_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // scale by factor of 2 about origin => extents from (0,0) to (20,10)
    let mut shape_copy = shape.clone();
    shape_copy.scale_mut(2.0);
    assert_extents_fuzzy_eq(&shape_copy, 0.0, 0.0, 20.0, 10.0, 1e-5);
}

#[test]
fn transform_builder_translate_test() {
    // Make a rectangle from (0,0) to (10,5)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 5.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // translate by (15, -2)
    let mut shape_copy = shape.clone();
    shape_copy.translate_mut(15.0, -2.0);
    // new bounding box => from (15,-2) to (25,3)
    assert_extents_fuzzy_eq(&shape_copy, 15.0, -2.0, 25.0, 3.0, 1e-5);
}

#[test]
fn transform_builder_center_test() {
    // Make a rectangle from (0,0) to (10,10) => extents from (0,0) to (10,10)
    let rect_pline = create_rectangle(0.0, 0.0, 10.0, 10.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // center => bounding box is from (-5,-5) to (5,5)
    let mut shape_copy = shape.clone();
    shape_copy.center_mut();
    assert_extents_fuzzy_eq(&shape_copy, -5.0, -5.0, 5.0, 5.0, 1e-5);
}

#[test]
fn transform_builder_mirror_x_test() {
    // rectangle from (1.0,1.0) to (4.0,3.0)
    let rect_pline = create_rectangle(1.0, 1.0, 4.0, 3.0);
    let shape = Shape::from_plines(vec![rect_pline]);

    // mirror_x => flip in place (y -> -y), so bounding box => min_y is -3.0, max_y is -1.0
    let mut shape_copy = shape.clone();
    shape_copy.mirror_x_mut();
    // bounding box => x from (1,4) to (1,4) unchanged, y from ( -3, -1)
    assert_extents_fuzzy_eq(&shape_copy, 1.0, -3.0, 4.0, -1.0, 1e-5);
}

#[test]
fn transform_builder_rotate_test() {
    // rectangle from (0.0,0.0) to (10.0, 0.0) => extents from (0,0) to (10,0) plus the small y range
    let mut rect_pline = Polyline::new_closed();
    rect_pline.add(0.0, 0.0, 0.0);
    rect_pline.add(10.0, 0.0, 0.0);
    rect_pline.add(10.0, 2.0, 0.0);
    rect_pline.add(0.0, 2.0, 0.0);

    let shape = Shape::from_plines(vec![rect_pline]);
    // rotate 90 degrees about (0,0)
    let mut shape_copy = shape.clone();
    shape_copy.rotate_mut(PI * 0.5);

    // after 90 deg rotation:
    //  originally x from [0..10], y from [0..2]
    //  becomes x from [-2..0], y from [0..10]
    assert_extents_fuzzy_eq(&shape_copy, -2.0, 0.0, 0.0, 10.0, 1e-5);
}

#[test]
fn shape_offset_simple_test() {
    // Make a single circle-like shape from an approximate circle.
    let circle_pline = create_approx_circle(0.0, 0.0, 5.0);
    let shape = Shape::from_plines(vec![circle_pline]);

    // Offset outward by 2; the two-arc circle should expand to roughly radius 7.
    let offset_opts = ShapeOffsetOptions::default();
    let result = shape.parallel_offset(-2.0, offset_opts);
    assert_eq!(result.ccw_plines.len(), 1);
    let final_bb = result.plines_index.bounds().unwrap();
    let min_x = final_bb.min_x;
    assert!(
        min_x < -6.95 && min_x > -7.05,
        "expected min_x ~ -7.0, got {min_x}"
    );
}
