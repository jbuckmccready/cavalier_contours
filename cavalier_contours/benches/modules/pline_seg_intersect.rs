use std::hint::black_box;

use cavalier_contours::{
    core::math::Vector2,
    polyline::{PlineVertex, pline_seg_intr},
};
use criterion::{BenchmarkId, Criterion};

fn arc_segment(
    center: Vector2<f64>,
    radius: f64,
    start_angle: f64,
    sweep: f64,
) -> (PlineVertex<f64>, PlineVertex<f64>) {
    let point_at_angle = |angle: f64| {
        let (sin, cos) = angle.sin_cos();
        center + Vector2::new(radius * cos, radius * sin)
    };
    (
        PlineVertex::from_vector2(point_at_angle(start_angle), (sweep / 4.0).tan()),
        PlineVertex::from_vector2(point_at_angle(start_angle + sweep), 0.0),
    )
}

pub fn pline_seg_intr_group(c: &mut Criterion) {
    let line = |start: Vector2<f64>, end: Vector2<f64>| {
        (
            PlineVertex::from_vector2(start, 0.0),
            PlineVertex::from_vector2(end, 0.0),
        )
    };
    let lower_semicircle = arc_segment(
        Vector2::new(0.0, 0.0),
        1.0,
        std::f64::consts::PI,
        std::f64::consts::PI,
    );
    let right_semicircle = arc_segment(
        Vector2::new(0.0, 0.0),
        1.0,
        -std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    );
    let left_semicircle = arc_segment(
        Vector2::new(1.0, 0.0),
        1.0,
        std::f64::consts::FRAC_PI_2,
        std::f64::consts::PI,
    );
    let identical_arc = arc_segment(Vector2::new(0.0, 0.0), 1.0, 0.25, 1.5);
    let partial_arc = arc_segment(Vector2::new(0.0, 0.0), 1.0, 1.0, 1.5);
    let disjoint_arc = arc_segment(Vector2::new(0.0, 0.0), 1.0, 3.0, 1.0);
    let cases = [
        (
            "line_line/true",
            line(Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0)),
            line(Vector2::new(-1.0, 1.0), Vector2::new(1.0, -1.0)),
        ),
        (
            "line_line/parallel",
            line(Vector2::new(-2.0, -1.0), Vector2::new(2.0, -1.0)),
            line(Vector2::new(-1.0, 5.0), Vector2::new(1.0, 5.0)),
        ),
        (
            "line_line/overlap",
            line(Vector2::new(-1.0, -1.0), Vector2::new(1.0, 1.0)),
            line(Vector2::new(0.0, 0.0), Vector2::new(0.5, 0.5)),
        ),
        (
            "line_arc/circle_miss",
            line(Vector2::new(-2.0, 2.0), Vector2::new(2.0, 2.0)),
            lower_semicircle,
        ),
        (
            "line_arc/tangent",
            line(Vector2::new(-2.0, -1.0), Vector2::new(2.0, -1.0)),
            lower_semicircle,
        ),
        (
            "line_arc/two_in_sweep",
            line(Vector2::new(-2.0, -0.5), Vector2::new(2.0, -0.5)),
            lower_semicircle,
        ),
        (
            "line_arc/one_in_sweep",
            line(Vector2::new(0.0, -2.0), Vector2::new(0.0, 2.0)),
            lower_semicircle,
        ),
        (
            "arc_line/two_in_sweep",
            lower_semicircle,
            line(Vector2::new(-2.0, -0.5), Vector2::new(2.0, -0.5)),
        ),
        (
            "arc_arc/no_intersect",
            lower_semicircle,
            arc_segment(Vector2::new(3.0, 0.0), 1.0, 0.0, std::f64::consts::PI),
        ),
        ("arc_arc/two_circle_hits", right_semicircle, left_semicircle),
        ("arc_arc/identical", identical_arc, identical_arc),
        ("arc_arc/partial_overlap", identical_arc, partial_arc),
        ("arc_arc/disjoint_sweeps", identical_arc, disjoint_arc),
    ];

    let mut group = c.benchmark_group("pline_seg_intr");
    for (name, (v1, v2), (u1, u2)) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, u1, u2),
            |b, &(v1, v2, u1, u2)| {
                b.iter(|| {
                    black_box(pline_seg_intr(
                        black_box(v1),
                        black_box(v2),
                        black_box(u1),
                        black_box(u2),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}
