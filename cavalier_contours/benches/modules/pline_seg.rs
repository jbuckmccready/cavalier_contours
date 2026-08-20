use std::hint::black_box;

use cavalier_contours::{
    core::math::Vector2,
    polyline::{
        PlineVertex, dist_from_segment_start, seg_arc_radius_and_center, seg_bounding_box,
        seg_closest_point, seg_distance_is_greater_than, seg_fast_approx_bounding_box, seg_length,
        seg_midpoint, seg_split_at_point, seg_tangent_vector,
    },
};
use criterion::{BenchmarkId, Criterion};

pub fn seg_midpoint_group(c: &mut Criterion) {
    let cases = [
        (
            "line",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "shallow_arc",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "quarter_circle",
            (
                PlineVertex::new(2.0, 2.0, std::f64::consts::FRAC_PI_8.tan()),
                PlineVertex::new(4.0, 4.0, 0.0),
            ),
        ),
        (
            "clockwise_semicircle",
            (
                PlineVertex::new(-123.5, 47.25, -1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_midpoint");
    for (name, vertices) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &vertices,
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_midpoint(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

pub fn seg_arc_radius_and_center_group(c: &mut Criterion) {
    let cases = [
        (
            "shallow_arc",
            PlineVertex::new(-123.5, 47.25, 1e-7),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
        (
            "quarter_circle",
            PlineVertex::new(1.0, 0.0, std::f64::consts::FRAC_PI_8.tan()),
            PlineVertex::new(0.0, 1.0, 0.0),
        ),
        (
            "semicircle",
            PlineVertex::new(-123.5, 47.25, 1.0),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
        (
            "clockwise_semicircle",
            PlineVertex::new(-123.5, 47.25, -1.0),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
    ];

    let mut group = c.benchmark_group("seg_arc_radius_and_center");
    for (name, v1, v2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2),
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_arc_radius_and_center(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

pub fn seg_tangent_vector_group(c: &mut Criterion) {
    let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
    let cases = [
        (
            "line",
            PlineVertex::new(-123.5, 47.25, 0.0),
            PlineVertex::new(891.75, -302.5, 0.0),
            Vector2::new(384.125, -127.625),
        ),
        (
            "shallow_arc_midpoint",
            PlineVertex::new(-123.5, 47.25, 1e-7),
            PlineVertex::new(891.75, -302.5, 0.0),
            Vector2::new(384.1249825125, -127.6250507625),
        ),
        (
            "quarter_circle_midpoint",
            PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
            PlineVertex::new(0.0, 1.0, 0.0),
            Vector2::new(
                std::f64::consts::FRAC_1_SQRT_2,
                std::f64::consts::FRAC_1_SQRT_2,
            ),
        ),
        (
            "semicircle_midpoint",
            PlineVertex::new(0.0, 0.0, 1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, -1.0),
        ),
        (
            "clockwise_semicircle_midpoint",
            PlineVertex::new(0.0, 0.0, -1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 1.0),
        ),
    ];

    let mut group = c.benchmark_group("seg_tangent_vector");
    for (name, v1, v2, point) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, point),
            |b, &(v1, v2, point)| {
                b.iter(|| {
                    black_box(seg_tangent_vector(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                    ))
                });
            },
        );
    }
    group.finish();
}

pub fn seg_fast_approx_bounding_box_group(c: &mut Criterion) {
    let cases = [
        (
            "line",
            PlineVertex::new(-123.5, 47.25, 0.0),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
        (
            "shallow_arc",
            PlineVertex::new(-123.5, 47.25, 1e-7),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
        (
            "quarter_circle",
            PlineVertex::new(1.0, 0.0, std::f64::consts::FRAC_PI_8.tan()),
            PlineVertex::new(0.0, 1.0, 0.0),
        ),
        (
            "clockwise_semicircle",
            PlineVertex::new(-123.5, 47.25, -1.0),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
    ];

    let mut group = c.benchmark_group("seg_fast_approx_bounding_box");
    for (name, v1, v2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2),
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_fast_approx_bounding_box(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

pub fn seg_bounding_box_group(c: &mut Criterion) {
    let arc = |start_angle: f64, sweep: f64| {
        let (start_sin, start_cos) = start_angle.sin_cos();
        let (end_sin, end_cos) = (start_angle + sweep).sin_cos();
        (
            PlineVertex::new(start_cos, start_sin, (sweep / 4.0).tan()),
            PlineVertex::new(end_cos, end_sin, 0.0),
        )
    };
    let cases = [
        (
            "line",
            PlineVertex::new(-123.5, 47.25, 0.0),
            PlineVertex::new(891.75, -302.5, 0.0),
        ),
        ("shallow_no_cardinal", arc(0.25, 4e-7).0, arc(0.25, 4e-7).1),
        (
            "quarter_no_cardinal",
            arc(0.1, std::f64::consts::FRAC_PI_2 * 0.8).0,
            arc(0.1, std::f64::consts::FRAC_PI_2 * 0.8).1,
        ),
        (
            "quarter_cardinal_endpoints",
            arc(0.0, std::f64::consts::FRAC_PI_2).0,
            arc(0.0, std::f64::consts::FRAC_PI_2).1,
        ),
        (
            "quarter_crosses_cardinal",
            arc(-std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_2).0,
            arc(-std::f64::consts::FRAC_PI_4, std::f64::consts::FRAC_PI_2).1,
        ),
        (
            "semicircle",
            arc(0.25, std::f64::consts::PI).0,
            arc(0.25, std::f64::consts::PI).1,
        ),
        (
            "clockwise_semicircle",
            arc(0.25, -std::f64::consts::PI).0,
            arc(0.25, -std::f64::consts::PI).1,
        ),
    ];

    let mut group = c.benchmark_group("seg_bounding_box");
    for (name, v1, v2) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2),
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_bounding_box(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

pub fn seg_closest_point_group(c: &mut Criterion) {
    let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
    let cases = [
        (
            "line_interior",
            PlineVertex::new(0.0, 0.0, 0.0),
            PlineVertex::new(4.0, 0.0, 0.0),
            Vector2::new(2.0, 3.0),
        ),
        (
            "line_outside",
            PlineVertex::new(0.0, 0.0, 0.0),
            PlineVertex::new(4.0, 0.0, 0.0),
            Vector2::new(5.0, 3.0),
        ),
        (
            "shallow_arc_projection",
            PlineVertex::new(-123.5, 47.25, 1e-7),
            PlineVertex::new(891.75, -302.5, 0.0),
            Vector2::new(384.0, -126.0),
        ),
        (
            "quarter_circle_projection",
            PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
            PlineVertex::new(0.0, 1.0, 0.0),
            Vector2::new(2.0, 2.0),
        ),
        (
            "semicircle_outside_sweep",
            PlineVertex::new(0.0, 0.0, 1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 2.0),
        ),
        (
            "semicircle_center",
            PlineVertex::new(0.0, 0.0, 1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 0.0),
        ),
        (
            "clockwise_semicircle_projection",
            PlineVertex::new(0.0, 0.0, -1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 2.0),
        ),
    ];

    let mut group = c.benchmark_group("seg_closest_point");
    for (name, v1, v2, point) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, point),
            |b, &(v1, v2, point)| {
                b.iter(|| {
                    black_box(seg_closest_point(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}

pub fn seg_distance_is_greater_than_group(c: &mut Criterion) {
    let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
    let cases = [
        (
            "line_interior",
            PlineVertex::new(0.0, 0.0, 0.0),
            PlineVertex::new(4.0, 0.0, 0.0),
            Vector2::new(2.0, 3.0),
            2.0,
        ),
        (
            "line_outside",
            PlineVertex::new(0.0, 0.0, 0.0),
            PlineVertex::new(4.0, 0.0, 0.0),
            Vector2::new(5.0, 3.0),
            4.0,
        ),
        (
            "shallow_arc_projection",
            PlineVertex::new(-123.5, 47.25, 1e-7),
            PlineVertex::new(891.75, -302.5, 0.0),
            Vector2::new(384.0, -126.0),
            1.0,
        ),
        (
            "quarter_circle_projection",
            PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
            PlineVertex::new(0.0, 1.0, 0.0),
            Vector2::new(2.0, 2.0),
            1.0,
        ),
        (
            "semicircle_outside_sweep",
            PlineVertex::new(0.0, 0.0, 1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 2.0),
            2.0,
        ),
        (
            "semicircle_center",
            PlineVertex::new(0.0, 0.0, 1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 0.0),
            0.5,
        ),
        (
            "clockwise_semicircle_projection",
            PlineVertex::new(0.0, 0.0, -1.0),
            PlineVertex::new(2.0, 0.0, 0.0),
            Vector2::new(1.0, 2.0),
            0.5,
        ),
    ];

    let mut group = c.benchmark_group("seg_distance_is_greater_than");
    for (name, v1, v2, point, distance) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, point, distance),
            |b, &(v1, v2, point, distance)| {
                b.iter(|| {
                    black_box(seg_distance_is_greater_than(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                        black_box(distance),
                        black_box(1e-5),
                    ))
                });
            },
        );
    }
    group.finish();
}

pub fn seg_split_at_point_group(c: &mut Criterion) {
    let quarter_circle_bulge = std::f64::consts::FRAC_PI_8.tan();
    let near_start_angle = std::f64::consts::PI * 1e-6;
    let cases = [
        (
            "line_interior",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
                Vector2::new(384.125, -127.625),
            ),
        ),
        (
            "arc_start",
            (
                PlineVertex::new(0.0, 0.0, 1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(0.0, 0.0),
            ),
        ),
        (
            "shallow_arc_midpoint",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
                Vector2::new(384.1249825125, -127.6250507625),
            ),
        ),
        (
            "quarter_circle_midpoint",
            (
                PlineVertex::new(1.0, 0.0, quarter_circle_bulge),
                PlineVertex::new(0.0, 1.0, 0.0),
                Vector2::new(
                    std::f64::consts::FRAC_1_SQRT_2,
                    std::f64::consts::FRAC_1_SQRT_2,
                ),
            ),
        ),
        (
            "semicircle_near_start",
            (
                PlineVertex::new(0.0, 0.0, 1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(1.0 - near_start_angle.cos(), -near_start_angle.sin()),
            ),
        ),
        (
            "clockwise_semicircle_midpoint",
            (
                PlineVertex::new(0.0, 0.0, -1.0),
                PlineVertex::new(2.0, 0.0, 0.0),
                Vector2::new(1.0, 1.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_split_at_point");
    for (name, input) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &input,
            |b, &(v1, v2, point)| {
                b.iter(|| {
                    black_box(seg_split_at_point(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                        black_box(1e-12),
                    ))
                });
            },
        );
    }
    group.finish();
}

pub fn seg_length_group(c: &mut Criterion) {
    let cases = [
        (
            "zero_length",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(-123.5, 47.25, 0.0),
            ),
        ),
        (
            "line",
            (
                PlineVertex::new(-123.5, 47.25, 0.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "shallow_arc",
            (
                PlineVertex::new(-123.5, 47.25, 1e-7),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "quarter_circle",
            (
                PlineVertex::new(1.0, 0.0, std::f64::consts::FRAC_PI_8.tan()),
                PlineVertex::new(0.0, 1.0, 0.0),
            ),
        ),
        (
            "semicircle",
            (
                PlineVertex::new(-123.5, 47.25, 1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
        (
            "clockwise_semicircle",
            (
                PlineVertex::new(-123.5, 47.25, -1.0),
                PlineVertex::new(891.75, -302.5, 0.0),
            ),
        ),
    ];

    let mut group = c.benchmark_group("seg_length");
    for (name, vertices) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &vertices,
            |b, &(v1, v2)| {
                b.iter(|| black_box(seg_length(black_box(v1), black_box(v2))));
            },
        );
    }
    group.finish();
}

pub fn dist_from_segment_start_group(c: &mut Criterion) {
    let open_arc = |start_angle: f64, sweep: f64| {
        let (start_sin, start_cos) = start_angle.sin_cos();
        let (end_sin, end_cos) = (start_angle + sweep).sin_cos();
        (
            PlineVertex::new(start_cos, start_sin, (sweep / 4.0).tan()),
            PlineVertex::new(end_cos, end_sin, 0.0),
        )
    };
    let arc_case = |name, sweep: f64, fraction: f64| {
        let start_angle = 0.25;
        let point_angle = start_angle + sweep * fraction;
        let (point_sin, point_cos) = point_angle.sin_cos();
        let (v1, v2) = open_arc(start_angle, sweep);
        (name, v1, v2, Vector2::new(point_cos, point_sin))
    };
    let cases = [
        (
            "line_interior",
            PlineVertex::new(-123.5, 47.25, 0.0),
            PlineVertex::new(891.75, -302.5, 0.0),
            Vector2::new(384.125, -127.625),
        ),
        arc_case("arc_start_snap", std::f64::consts::PI, 0.0),
        arc_case("arc_end_snap", std::f64::consts::PI, 1.0),
        arc_case("shallow_arc_midpoint", 4e-7, 0.5),
        arc_case("quarter_circle_midpoint", std::f64::consts::FRAC_PI_2, 0.5),
        arc_case("semicircle_near_start", std::f64::consts::PI, 1e-6),
        arc_case("semicircle_midpoint", std::f64::consts::PI, 0.5),
        arc_case("semicircle_near_end", std::f64::consts::PI, 1.0 - 1e-6),
        arc_case("clockwise_semicircle_midpoint", -std::f64::consts::PI, 0.5),
    ];

    let mut group = c.benchmark_group("dist_from_segment_start");
    for (name, v1, v2, point) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &(v1, v2, point),
            |b, &(v1, v2, point)| {
                b.iter(|| {
                    black_box(dist_from_segment_start(
                        black_box(v1),
                        black_box(v2),
                        black_box(point),
                        black_box(1e-12),
                    ))
                });
            },
        );
    }
    group.finish();
}
