use std::hint::black_box;

use cavalier_contours::{
    core::math::Vector2,
    polyline::{PlineSource, PlineSourceMut, Polyline},
};
use criterion::{Bencher, BenchmarkId, Criterion};

use super::test_polylines::pathological1;

fn bench_polyline_area(b: &mut Bencher<'_>, polyline: &Polyline<f64>) {
    b.iter(|| polyline.area());
}

pub fn polyline_area_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_area");
    let vertex_counts = &[25, 250, 2500, 25000];
    for &i in vertex_counts {
        group.bench_with_input(BenchmarkId::new("pathological_area", i), &i, |b, i| {
            bench_polyline_area(b, &pathological1(*i));
        });
    }

    group.finish();
}

fn regular_polygon(vertex_count: u32, bulge: f64) -> Polyline<f64> {
    let mut result = Polyline::new_closed();
    result.reserve(usize::try_from(vertex_count).unwrap());
    for i in 0..vertex_count {
        let angle = f64::from(i) * std::f64::consts::TAU / f64::from(vertex_count);
        let (sin, cos) = angle.sin_cos();
        result.add(40.0 * cos, 40.0 * sin, bulge);
    }
    result
}

pub fn polyline_winding_number_group(c: &mut Criterion) {
    const VERTEX_COUNT: u32 = 1000;
    let point = Vector2::new(0.0, 0.0);
    let lines = regular_polygon(VERTEX_COUNT, 0.0);
    let circle_arcs = regular_polygon(
        VERTEX_COUNT,
        (std::f64::consts::PI / (2.0 * f64::from(VERTEX_COUNT))).tan(),
    );

    // Each arc chord crosses the horizontal ray from `point` and takes the point-in-circle path.
    let mut crossing_arcs = Polyline::new_closed();
    crossing_arcs.reserve(usize::try_from(VERTEX_COUNT).unwrap());
    for i in 0..VERTEX_COUNT {
        if i % 2 == 0 {
            crossing_arcs.add(10.0, -1.0, -1.0);
        } else {
            crossing_arcs.add(10.0, 1.0, 1.0);
        }
    }

    let mut group = c.benchmark_group("polyline_winding_number");
    group.bench_function("lines", |b| {
        b.iter(|| black_box(lines.winding_number(black_box(point))));
    });
    group.bench_function("circle_arcs", |b| {
        b.iter(|| black_box(circle_arcs.winding_number(black_box(point))));
    });
    group.bench_function("arc_distance_checks", |b| {
        b.iter(|| black_box(crossing_arcs.winding_number(black_box(point))));
    });
    group.finish();
}
