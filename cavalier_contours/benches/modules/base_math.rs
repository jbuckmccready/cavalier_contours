use std::hint::black_box;

use cavalier_contours::core::math::{Vector2, delta_angle, point_within_arc_sweep};
use criterion::{BenchmarkId, Criterion};

pub fn point_within_arc_sweep_group(c: &mut Criterion) {
    let center = Vector2::new(0.0, 0.0);
    let start = Vector2::new(10.0, 0.0);
    let end = Vector2::new(0.0, 10.0);
    let cases = [
        ("interior", Vector2::new(5.0, 5.0)),
        ("exterior", Vector2::new(-5.0, -5.0)),
        ("near_start_ray", Vector2::new(5.0, -5e-6)),
        ("near_end_ray", Vector2::new(-5e-6, 5.0)),
        ("behind_start_ray", Vector2::new(-5.0, 5e-6)),
        ("near_center", Vector2::new(1e-6, 1e-6)),
    ];

    let mut group = c.benchmark_group("point_within_arc_sweep");
    for (name, point) in cases {
        group.bench_with_input(BenchmarkId::from_parameter(name), &point, |b, &point| {
            b.iter(|| {
                black_box(point_within_arc_sweep(
                    black_box(center),
                    black_box(start),
                    black_box(end),
                    black_box(false),
                    black_box(point),
                    black_box(1e-5),
                ))
            });
        });
    }
    group.finish();
}

pub fn delta_angle_group(c: &mut Criterion) {
    let cases = [
        ("same", (0.25, 0.25)),
        ("small_positive", (0.25, 0.5)),
        ("small_negative", (0.5, 0.25)),
        ("wrap_positive", (6.0, 0.25)),
        ("wrap_negative", (0.25, 6.0)),
        ("large_angles", (1000.0, -1000.0)),
    ];

    let mut group = c.benchmark_group("delta_angle");
    for (name, angles) in cases {
        group.bench_with_input(
            BenchmarkId::from_parameter(name),
            &angles,
            |b, &(angle1, angle2)| {
                b.iter(|| black_box(delta_angle(black_box(angle1), black_box(angle2))));
            },
        );
    }
    group.finish();
}
