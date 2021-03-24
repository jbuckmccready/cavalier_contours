use cavalier_contours::{
    assert_fuzzy_eq,
    core::traits::FuzzyEq,
    polyline::{PlineVertex, Polyline},
};
use std::{borrow::Cow, f64::consts::PI};

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
        let polyline = Polyline::new_closed();
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert!(result.is_empty());
        assert!(result.is_closed());
    }

    {
        let mut polyline = Polyline::new_closed();
        polyline.add(2.0, 2.0, 0.5);
        let result = polyline.remove_repeat_pos(1e-5);
        assert!(matches!(result, Cow::Borrowed(_)));
        assert_eq!(result.len(), 1);
        assert!(result.is_closed());
        assert_fuzzy_eq!(result[0], PlineVertex::new(2.0, 2.0, 0.5));
    }

    {
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
        let mut square = Polyline::new_closed();
        square.add(0.0, 0.0, 0.0);
        square.add(2.0, 0.0, 0.0);
        square.add(2.0, 2.0, 0.0);
        square.add(0.0, 2.0, 0.0);
        assert_fuzzy_eq!(square.area(), 4.0);
        square.invert_direction();
        assert_fuzzy_eq!(square.area(), -4.0);
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
