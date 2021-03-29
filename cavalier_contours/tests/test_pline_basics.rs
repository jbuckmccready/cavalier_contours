use cavalier_contours::{
    assert_fuzzy_eq,
    core::{math::bulge_from_angle, traits::FuzzyEq},
    polyline::{PlineVertex, Polyline},
};
use std::{
    borrow::Cow,
    f64::consts::{PI, TAU},
};

#[test]
fn iter_segments() {
    let mut polyline = Polyline::<f64>::new();
    assert_eq!(polyline.iter_segments().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segments().collect::<Vec<_>>().len(), 0);

    polyline.add(1.0, 2.0, 0.3);
    assert_eq!(polyline.iter_segments().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segments().collect::<Vec<_>>().len(), 0);

    polyline.add(4.0, 5.0, 0.6);
    assert_eq!(polyline.iter_segments().size_hint(), (1, Some(1)));
    let one_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(one_seg.len(), 1);
    assert_eq!(one_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(one_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));

    polyline.set_is_closed(true);
    assert_eq!(polyline.iter_segments().size_hint(), (2, Some(2)));
    let two_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(two_seg.len(), 2);
    assert_eq!(two_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(two_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg[1].1, PlineVertex::new(1.0, 2.0, 0.3));

    polyline.add(0.5, 0.5, 0.5);
    assert_eq!(polyline.iter_segments().size_hint(), (3, Some(3)));
    let three_seg = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(three_seg.len(), 3);
    assert_eq!(three_seg[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(three_seg[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(three_seg[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(three_seg[1].1, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(three_seg[2].0, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(three_seg[2].1, PlineVertex::new(1.0, 2.0, 0.3));

    polyline.set_is_closed(false);
    assert_eq!(polyline.iter_segments().size_hint(), (2, Some(2)));
    let two_seg_open = polyline.iter_segments().collect::<Vec<_>>();
    assert_eq!(two_seg_open.len(), 2);
    assert_eq!(two_seg_open[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(two_seg_open[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg_open[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(two_seg_open[1].1, PlineVertex::new(0.5, 0.5, 0.5));
}

#[test]
fn visit_segments() {
    let mut polyline = Polyline::<f64>::new();
    let mut results = Vec::new();
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });

    assert_eq!(results.len(), 0);

    polyline.add(1.0, 2.0, 0.3);
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });
    assert_eq!(results.len(), 0);

    polyline.add(4.0, 5.0, 0.6);
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });
    assert_eq!(results.len(), 1);
    assert_eq!(results[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(results[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    results.clear();

    polyline.set_is_closed(true);
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });
    assert_eq!(results.len(), 2);
    assert_eq!(results[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(results[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].1, PlineVertex::new(1.0, 2.0, 0.3));
    results.clear();

    polyline.add(0.5, 0.5, 0.5);
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });
    assert_eq!(results.len(), 3);
    assert_eq!(results[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(results[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].1, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(results[2].0, PlineVertex::new(0.5, 0.5, 0.5));
    assert_eq!(results[2].1, PlineVertex::new(1.0, 2.0, 0.3));
    results.clear();

    polyline.set_is_closed(false);
    polyline.visit_segments(&mut |v1, v2| {
        results.push((v1, v2));
        true
    });
    assert_eq!(results[0].0, PlineVertex::new(1.0, 2.0, 0.3));
    assert_eq!(results[0].1, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].0, PlineVertex::new(4.0, 5.0, 0.6));
    assert_eq!(results[1].1, PlineVertex::new(0.5, 0.5, 0.5));
}

#[test]
fn iter_segment_indexes() {
    let mut polyline = Polyline::<f64>::new();
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segment_indexes().collect::<Vec<_>>().len(), 0);

    polyline.add(1.0, 2.0, 0.3);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (0, Some(0)));
    assert_eq!(polyline.iter_segment_indexes().collect::<Vec<_>>().len(), 0);

    polyline.add(4.0, 5.0, 0.6);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (1, Some(1)));
    let one_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(one_seg, vec![(0, 1)]);

    polyline.set_is_closed(true);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (2, Some(2)));
    let two_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(two_seg, vec![(0, 1), (1, 0)]);

    polyline.add(0.5, 0.5, 0.5);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (3, Some(3)));
    let three_seg = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(three_seg, vec![(0, 1), (1, 2), (2, 0)]);

    polyline.set_is_closed(false);
    assert_eq!(polyline.iter_segment_indexes().size_hint(), (2, Some(2)));
    let two_seg_open = polyline.iter_segment_indexes().collect::<Vec<_>>();
    assert_eq!(two_seg_open, vec![(0, 1), (1, 2)]);
}

#[test]
fn invert_direction() {
    let mut polyline = Polyline::new_closed();
    polyline.add(0.0, 0.0, 0.1);
    polyline.add(2.0, 0.0, 0.2);
    polyline.add(2.0, 2.0, 0.3);
    polyline.add(0.0, 2.0, 0.4);

    polyline.invert_direction();

    assert_fuzzy_eq!(polyline[0], PlineVertex::new(0.0, 2.0, -0.3));
    assert_fuzzy_eq!(polyline[1], PlineVertex::new(2.0, 2.0, -0.2));
    assert_fuzzy_eq!(polyline[2], PlineVertex::new(2.0, 0.0, -0.1));
    assert_fuzzy_eq!(polyline[3], PlineVertex::new(0.0, 0.0, -0.4));
}

#[test]
fn remove_repeat() {
    {
        // empty polyline
        let polyline = Polyline::new_closed();
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert!(result.is_empty());
        assert!(result.is_closed());
    }

    {
        // single vertex
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 1);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
    }

    {
        // two repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // two repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // no repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // no repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(4.0, 3.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(4.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(2.0, 2.0, 1.0));
    }
}

#[test]
fn remove_redundant_removes_repeat_pos() {
    {
        // empty polyline
        let polyline = Polyline::new_closed();
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert!(result.is_empty());
        assert!(result.is_closed());
    }

    {
        // single vertex
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 1);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
    }

    {
        // two repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // two repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(2.0, 2.0, 1.0);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(3.0, 3.0, 0.5);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.5));
    }

    {
        // no repeats, closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // no repeats, open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(4.0, 3.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(4.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for closed polyline
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
    }

    {
        // last repeats position on first for open polyline
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.5);
        polyline.add(3.0, 3.0, 1.0);
        polyline.add(2.0, 2.0, 1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(2.0, 2.0, 1.0));
    }
}

#[test]
fn remove_redundant() {
    {
        // redundant point on line and repeat position
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(4.0, 4.0, 0.0);
        polyline.add(2.0, 4.0, 0.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 3);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(4.0, 4.0, 0.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(2.0, 4.0, 0.0));
    }

    {
        // self intersecting points along line (collinear but opposing direction, points should not
        // be removed)
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(2.5, 2.5, 0.0);
        polyline.add(4.0, 4.0, 0.0);
        polyline.add(2.0, 4.0, 0.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 5);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(2.5, 2.5, 0.0));
        assert_fuzzy_eq!(result[3], PlineVertex::new(4.0, 4.0, 0.0));
        assert_fuzzy_eq!(result[4], PlineVertex::new(2.0, 4.0, 0.0));
    }

    {
        // simple counter clockwise circle with extra vertex along one arc
        let bulge = (std::f64::consts::FRAC_PI_2 / 4.0).tan();
        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, 0.0, -bulge);
        polyline.add(1.0, 1.0, -bulge);
        polyline.add(2.0, 0.0, -1.0);
        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(0.0, 0.0, -1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(2.0, 0.0, -1.0));
    }

    {
        // arcs along greater arc
        let radius = 5.0;
        let max_angle = std::f64::consts::FRAC_PI_2;
        let count = 4;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new();
        (0..=count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(
            result[0],
            PlineVertex::new(radius, 0.0, bulge_from_angle(max_angle))
        );
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.0, radius, bulge));
    }

    {
        // arcs along circle
        let radius = 5.0;
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new_closed();
        (0..count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(-radius, 0.0, 1.0));
    }

    {
        // arcs along circle open polyline
        let radius = 5.0;
        let max_angle = std::f64::consts::TAU;
        let count = 10;
        let sub_angle = (1.0 / count as f64) * max_angle;
        let bulge = bulge_from_angle(sub_angle);

        let mut polyline = Polyline::new();
        (0..=count)
            .map(|i| (i as f64) * sub_angle)
            .for_each(|angle| polyline.add(radius * angle.cos(), radius * angle.sin(), bulge));

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(-radius, 0.0, 1.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(radius, 0.0, bulge));
    }

    {
        // already minimum circle
        let radius = 5.0;

        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, -radius, 1.0);
        polyline.add(0.0, radius, 1.0);

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(0.0, -radius, 1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(0.0, radius, 1.0));
    }

    {
        // closed half circle with arc that causes first vertex to be redundant
        let radius = 5.0;

        let bulge = bulge_from_angle(-std::f64::consts::FRAC_PI_2);

        let mut polyline = Polyline::new_closed();
        polyline.add(0.0, radius, bulge);
        polyline.add(radius, 0.0, 0.0);
        polyline.add(-radius, 0.0, bulge);

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 2);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-radius, 0.0, -1.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(radius, 0.0, 0.0));
    }

    {
        // open polyline with bulge values that would cause first vertex to be redundant if
        // polyline were closed
        let radius = 5.0;

        let bulge = bulge_from_angle(-std::f64::consts::FRAC_PI_2);

        let mut polyline = Polyline::new();
        polyline.add(0.0, radius, bulge);
        polyline.add(radius, 0.0, 0.0);
        polyline.add(-radius, 0.0, bulge);

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 3);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], polyline[0]);
        assert_fuzzy_eq!(result[1], polyline[1]);
        assert_fuzzy_eq!(result[2], polyline[2]);
    }

    {
        // closed path with redundant first vertex point along line
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, -2.0, 0.0);
        polyline.add(-2.0, -2.0, 0.0);
        polyline.add(-1.0, -1.0, 0.0);

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Owned(_)));
        assert_eq!(result.len(), 3);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(-2.0, -2.0, 0.0));
        assert_fuzzy_eq!(result[1], PlineVertex::new(3.0, 3.0, 0.0));
        assert_fuzzy_eq!(result[2], PlineVertex::new(3.0, -2.0, 0.0));
    }

    {
        // open polyline with values that would cause first vertex to be redundant due to being
        // collinear if polyline were closed
        let mut polyline = Polyline::new();
        polyline.add(2.0, 2.0, 0.0);
        polyline.add(3.0, 3.0, 0.0);
        polyline.add(3.0, -2.0, 0.0);
        polyline.add(-2.0, -2.0, 0.0);
        polyline.add(-1.0, -1.0, 0.0);

        let result = polyline.remove_redundant(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 5);
        assert!(!result.is_closed());
        assert_fuzzy_eq!(result[0], polyline[0]);
        assert_fuzzy_eq!(result[1], polyline[1]);
        assert_fuzzy_eq!(result[2], polyline[2]);
        assert_fuzzy_eq!(result[3], polyline[3]);
        assert_fuzzy_eq!(result[4], polyline[4]);
    }
}

#[test]
fn area() {
    {
        let mut circle = Polyline::new_closed();
        circle.add(0.0, 0.0, 1.0);
        circle.add(2.0, 0.0, 1.0);
        assert_fuzzy_eq!(circle.area(), PI);
        circle.invert_direction();
        assert_fuzzy_eq!(circle.area(), -PI);
    }

    {
        let mut half_circle = Polyline::new_closed();
        half_circle.add(0.0, 0.0, -1.0);
        half_circle.add(2.0, 0.0, 0.0);
        assert_fuzzy_eq!(half_circle.area(), -0.5 * PI);
        half_circle.invert_direction();
        assert_fuzzy_eq!(half_circle.area(), 0.5 * PI);
    }

    {
        let mut rectangle = Polyline::new_closed();
        rectangle.add(0.0, 0.0, 0.0);
        rectangle.add(3.0, 0.0, 0.0);
        rectangle.add(3.0, 2.0, 0.0);
        rectangle.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(rectangle.area(), 6.0);
        rectangle.invert_direction();
        assert_fuzzy_eq!(rectangle.area(), -6.0);
    }

    {
        let mut open_polyline = Polyline::new();
        open_polyline.add(0.0, 0.0, 0.0);
        open_polyline.add(2.0, 0.0, 0.0);
        open_polyline.add(2.0, 2.0, 0.0);
        open_polyline.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(open_polyline.area(), 0.0);
        open_polyline.invert_direction();
        assert_fuzzy_eq!(open_polyline.area(), 0.0);
    }

    {
        let empty_open_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_open_polyline.area(), 0.0);
    }

    {
        let empty_closed_polyline = Polyline::<f64>::new_closed();
        assert_fuzzy_eq!(empty_closed_polyline.area(), 0.0);
    }

    {
        let mut one_vertex_open_polyline = Polyline::<f64>::new();
        one_vertex_open_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_open_polyline.area(), 0.0);
    }

    {
        let mut one_vertex_closed_polyline = Polyline::<f64>::new_closed();
        one_vertex_closed_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_closed_polyline.area(), 0.0);
    }
}

#[test]
fn path_length() {
    {
        let empty_open_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_open_polyline.path_length(), 0.0);
    }

    {
        let empty_closed_polyline = Polyline::<f64>::new();
        assert_fuzzy_eq!(empty_closed_polyline.path_length(), 0.0);
    }

    {
        let mut one_vertex_open_polyline = Polyline::<f64>::new();
        one_vertex_open_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_open_polyline.path_length(), 0.0);
    }

    {
        let mut one_vertex_closed_polyline = Polyline::<f64>::new_closed();
        one_vertex_closed_polyline.add(1.0, 1.0, 0.0);
        assert_fuzzy_eq!(one_vertex_closed_polyline.path_length(), 0.0);
    }

    {
        let mut circle = Polyline::new_closed();
        circle.add(0.0, 0.0, 1.0);
        circle.add(2.0, 0.0, 1.0);
        assert_fuzzy_eq!(circle.path_length(), TAU);
        circle.invert_direction();
        assert_fuzzy_eq!(circle.path_length(), TAU);
    }

    {
        let mut half_circle = Polyline::new_closed();
        half_circle.add(0.0, 0.0, -1.0);
        half_circle.add(2.0, 0.0, 0.0);
        assert_fuzzy_eq!(half_circle.path_length(), PI + 2.0);
        half_circle.invert_direction();
        assert_fuzzy_eq!(half_circle.path_length(), PI + 2.0);
    }

    {
        let mut rectangle = Polyline::new_closed();
        rectangle.add(0.0, 0.0, 0.0);
        rectangle.add(3.0, 0.0, 0.0);
        rectangle.add(3.0, 2.0, 0.0);
        rectangle.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(rectangle.path_length(), 10.0);
        rectangle.invert_direction();
        assert_fuzzy_eq!(rectangle.path_length(), 10.0);
    }

    {
        let mut open_polyline = Polyline::new();
        open_polyline.add(0.0, 0.0, 0.0);
        open_polyline.add(3.0, 0.0, 0.0);
        open_polyline.add(3.0, 2.0, 0.0);
        open_polyline.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(open_polyline.path_length(), 8.0);
        open_polyline.invert_direction();
        assert_fuzzy_eq!(open_polyline.path_length(), 8.0);
    }
}
