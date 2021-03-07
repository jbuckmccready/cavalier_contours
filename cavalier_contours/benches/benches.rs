use cavalier_contours::*;
use criterion::{criterion_group, criterion_main, Bencher, BenchmarkId, Criterion};
mod test_polylines;
use test_polylines::*;

fn bench_polyline_area(b: &mut Bencher, polyline: &Polyline<f64>) {
    b.iter(|| {
        polyline.area();
    })
}

fn polyline_area_group(c: &mut Criterion) {
    let mut group = c.benchmark_group("polyline_area");
    let vertex_counts = &[25, 250, 2500, 25000];
    for &i in vertex_counts {
        group.bench_with_input(BenchmarkId::new("pathological_area", i), &i, |b, i| {
            bench_polyline_area(b, &pathological1(*i))
        });
    }

    group.finish();
}

criterion_group!(polyline_area, polyline_area_group,);
criterion_main!(polyline_area);
