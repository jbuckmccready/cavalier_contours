use std::hint::black_box;

use cavalier_contours::polyline::{
    PlineSource, Polyline, internal::pline_offset::create_raw_offset,
};
use criterion::{Bencher, BenchmarkId, Criterion, criterion_group, criterion_main};

mod test_polylines;

use test_polylines::{
    bezier_enclosure, floor_plan, involute_gear, involute_gear_with_arcs, mechanical_bracket,
    pathological1, pathological1_no_arcs, profile1, profile1_no_arcs, profile2, profile2_no_arcs,
    road_centerline,
};

fn bench_polyline_area(b: &mut Bencher<'_>, polyline: &Polyline<f64>) {
    b.iter(|| polyline.area());
}

fn polyline_area_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_area");
    let vertex_counts = &[25, 250, 2500, 25000];
    for &i in vertex_counts {
        group.bench_with_input(BenchmarkId::new("pathological_area", i), &i, |b, i| {
            bench_polyline_area(b, &pathological1(*i));
        });
    }

    group.finish();
}

fn repeat_offsets(polyline: &Polyline<f64>, offset: f64, count: u32) {
    for i in 1..=count {
        let offset = f64::from(i) * offset;
        black_box(polyline.parallel_offset(black_box(offset)));
        black_box(polyline.parallel_offset(black_box(-offset)));
    }
}

fn repeat_raw_offsets(polyline: &Polyline<f64>, offset: f64, count: u32) {
    for i in 1..=count {
        let offset = f64::from(i) * offset;
        black_box(create_raw_offset::<_, _, Polyline<f64>>(
            polyline,
            black_box(offset),
            1e-5,
        ));
        black_box(create_raw_offset::<_, _, Polyline<f64>>(
            polyline,
            black_box(-offset),
            1e-5,
        ));
    }
}

fn raw_offset_creation_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("raw_offset_creation");

    let profile1 = profile1();
    group.bench_function("profile1", |b| {
        b.iter(|| repeat_raw_offsets(&profile1, 0.1, 40));
    });

    let profile2 = profile2();
    group.bench_function("profile2", |b| {
        b.iter(|| repeat_raw_offsets(&profile2, 0.1, 40));
    });

    let profile1_no_arcs = profile1_no_arcs();
    group.bench_function("profile1_no_arcs", |b| {
        b.iter(|| repeat_raw_offsets(&profile1_no_arcs, 0.1, 40));
    });

    let profile2_no_arcs = profile2_no_arcs();
    group.bench_function("profile2_no_arcs", |b| {
        b.iter(|| repeat_raw_offsets(&profile2_no_arcs, 0.1, 40));
    });

    let floor_plan = floor_plan();
    group.bench_function("floor_plan", |b| {
        b.iter(|| repeat_raw_offsets(&floor_plan, 0.25, 12));
    });

    let mechanical_bracket = mechanical_bracket();
    group.bench_function("mechanical_bracket", |b| {
        b.iter(|| repeat_raw_offsets(&mechanical_bracket, 0.2, 10));
    });

    let road_centerline = road_centerline();
    group.bench_function("road_centerline", |b| {
        b.iter(|| repeat_raw_offsets(&road_centerline, 0.5, 8));
    });

    let bezier_enclosure = bezier_enclosure();
    group.bench_function("bezier_enclosure", |b| {
        b.iter(|| repeat_raw_offsets(&bezier_enclosure, 0.25, 8));
    });

    let involute_gear = involute_gear();
    group.bench_function("involute_gear", |b| {
        b.iter(|| repeat_raw_offsets(&involute_gear, 0.2, 4));
    });

    let involute_gear_with_arcs = involute_gear_with_arcs();
    group.bench_function("involute_gear_with_arcs", |b| {
        b.iter(|| repeat_raw_offsets(&involute_gear_with_arcs, 0.2, 4));
    });

    let pathological1 = pathological1(100);
    group.bench_function("pathological1", |b| {
        b.iter(|| repeat_raw_offsets(&pathological1, 1.0, 30));
    });

    let pathological1_no_arcs = pathological1_no_arcs(100);
    group.bench_function("pathological1_no_arcs", |b| {
        b.iter(|| repeat_raw_offsets(&pathological1_no_arcs, 1.0, 30));
    });

    group.finish();
}

fn polyline_offset_group(c: &mut Criterion) {
    let profile1 = profile1();
    c.bench_function("profile1", |b| {
        b.iter(|| repeat_offsets(&profile1, 0.1, 40));
    });

    let profile2 = profile2();
    c.bench_function("profile2", |b| {
        b.iter(|| repeat_offsets(&profile2, 0.1, 40));
    });

    let profile1_no_arcs = profile1_no_arcs();
    c.bench_function("profile1_no_arcs", |b| {
        b.iter(|| repeat_offsets(&profile1_no_arcs, 0.1, 40));
    });

    let profile2_no_arcs = profile2_no_arcs();
    c.bench_function("profile2_no_arcs", |b| {
        b.iter(|| repeat_offsets(&profile2_no_arcs, 0.1, 40));
    });

    let floor_plan = floor_plan();
    c.bench_function("floor_plan", |b| {
        b.iter(|| repeat_offsets(&floor_plan, 0.25, 12));
    });

    let mechanical_bracket = mechanical_bracket();
    c.bench_function("mechanical_bracket", |b| {
        b.iter(|| repeat_offsets(&mechanical_bracket, 0.2, 10));
    });

    let road_centerline = road_centerline();
    c.bench_function("road_centerline", |b| {
        b.iter(|| repeat_offsets(&road_centerline, 0.5, 8));
    });

    let bezier_enclosure = bezier_enclosure();
    c.bench_function("bezier_enclosure", |b| {
        b.iter(|| repeat_offsets(&bezier_enclosure, 0.25, 8));
    });

    let involute_gear = involute_gear();
    c.bench_function("involute_gear", |b| {
        b.iter(|| repeat_offsets(&involute_gear, 0.2, 4));
    });

    let involute_gear_with_arcs = involute_gear_with_arcs();
    c.bench_function("involute_gear_with_arcs", |b| {
        b.iter(|| repeat_offsets(&involute_gear_with_arcs, 0.2, 4));
    });

    let pathological1 = pathological1(100);
    c.bench_function("pathological1", |b| {
        b.iter(|| repeat_offsets(&pathological1, 1.0, 30));
    });

    let pathological1_no_arcs = pathological1_no_arcs(100);
    c.bench_function("pathological1_no_arcs", |b| {
        b.iter(|| repeat_offsets(&pathological1_no_arcs, 1.0, 30));
    });
}

criterion_group!(
    benches,
    polyline_area_group,
    raw_offset_creation_group,
    polyline_offset_group
);
criterion_main!(benches);
