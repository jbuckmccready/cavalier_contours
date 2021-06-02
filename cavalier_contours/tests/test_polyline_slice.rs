use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

use cavalier_contours::{
    assert_fuzzy_eq,
    core::traits::FuzzyEq,
    core::{
        math::{bulge_from_angle, point_on_circle, Vector2},
        Control,
    },
    pline_closed, pline_open,
    polyline::{seg_length, OpenPlineSlice, PlineVertex, Polyline, PolylineSlice},
};

const POS_EQ_EPS: f64 = 1e-5;

#[test]
fn from_slice_points_single_seg() {
    let pline = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];

    // complete polyline
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // complete polyline (end segment index on top of final vertex)
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // slice from start to middle
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end (end segment index on top of final vertex)
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from first third to second third of the segment
    {
        let start_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + FRAC_PI_3);
        let end_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + 2.0 * FRAC_PI_3);
        let slice =
            OpenPlineSlice::from_slice_points(&pline, start_point, 0, end_point, 0, POS_EQ_EPS)
                .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_3);
        let expected_result = {
            let mut p = Polyline::new();
            p.add_vertex(PlineVertex::from_vector2(start_point, bulge));
            p.add_vertex(PlineVertex::from_vector2(end_point, 0.0));
            p
        };

        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // collapsed slice at start
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        );

        assert!(slice.is_none());
    }

    // collapsed slice at end
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        );

        assert!(slice.is_none());
    }
}

#[test]
fn from_slice_points_multi_seg() {
    let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
    // complete polyline
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // complete polyline (end segment index on top of last vertex)
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            2,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // slice from start to middle of first segment
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end of first segment
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to second vertex
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to middle of second segment
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            1,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment (using previous index for start)
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of first segment to last vertex position
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of end segment to middle of first segment (wrapping)
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = slice.to_polyline(&pline, POS_EQ_EPS);
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![
            (1.0, 0.5, 0.0),
            (1.0, 1.0, 0.0),
            (0.0, 0.0, bulge),
            (0.5, -0.5, 0.0)
        ];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // collapsed slice at start
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.0, 0.0),
            0,
            POS_EQ_EPS,
        );
        assert!(slice.is_none());
    }

    // collapsed slice at midpoint of second segment
    {
        let slice = OpenPlineSlice::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        );
        assert!(slice.is_none());
    }
}

macro_rules! assert_path_length_result_eq {
    ($left:expr, $right:expr) => {
        match ($left, $right) {
            (Ok((index1, point1)), Ok((index2, point2)))
                if index1 == index2 && point1.fuzzy_eq_eps(point2, POS_EQ_EPS) => {}
            (Err(path_length1), Err(path_length2))
                if path_length1.fuzzy_eq_eps(path_length2, POS_EQ_EPS) => {}
            _ => panic!(
                "result cases do not match: left: {:?}, right: {:?}",
                $left, $right
            ),
        }
    };
}

#[test]
fn visit_segs() {
    let pline = pline_closed![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
    let slice = OpenPlineSlice::from_entire_pline(&pline);

    // visit all
    {
        let mut result = Vec::new();
        let mut visitor = |v1: PlineVertex<f64>, v2: PlineVertex<f64>| {
            result.push((v1, v2));
        };

        slice.visit_segs(&pline, &mut visitor);

        let expected = {
            let mut segs = pline.iter_segments().collect::<Vec<_>>();
            // last vertex bulge is set to zero when iterating the slice so set it to zero to match
            // (not a meaningful value)
            segs.last_mut().unwrap().1.bulge = 0.0;
            segs
        };

        assert_eq!(result.len(), expected.len());

        for ((v1, v2), (u1, u2)) in result.iter().zip(expected) {
            assert_fuzzy_eq!(v1, u1);
            assert_fuzzy_eq!(v2, u2);
        }
    }

    // visit one
    {
        let mut result = Vec::new();
        let mut visitor = |v1: PlineVertex<f64>, v2: PlineVertex<f64>| {
            result.push((v1, v2));
            Control::Break(())
        };

        slice.visit_segs(&pline, &mut visitor);

        assert_eq!(result.len(), 1);

        assert_fuzzy_eq!(result[0].0, pline[0]);
        assert_fuzzy_eq!(result[0].1, pline[1]);
    }
}

#[test]
fn find_point_at_path_length() {
    let pline = pline_closed![
        (0.0, 0.0, 1.0),
        (1.0, 0.0, -1.0),
        (1.0, 1.0, 0.0),
        (1.0, 2.0, 0.0)
    ];
    let pline_path_length = pline.path_length();
    let slice = OpenPlineSlice::from_entire_pline(&pline);

    // 0 path length (point at very start)
    {
        let r = slice.find_point_at_path_length(&pline, 0.0);
        let expected = Ok((0, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // total path length (point at very end)
    {
        let r = slice.find_point_at_path_length(&pline, pline_path_length);
        let expected = Ok((3, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // negative path length
    {
        let r = slice.find_point_at_path_length(&pline, -1.0);
        let expected = Ok((0, Vector2::new(0.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // target path length greater than total
    {
        let r = slice.find_point_at_path_length(&pline, pline_path_length + 1.0);
        let expected = Err(pline_path_length);
        assert_path_length_result_eq!(r, expected);
    }

    // half path length of first seg
    {
        let target_path_length = seg_length(pline[0], pline[1]) / 2.0;
        let r = slice.find_point_at_path_length(&pline, target_path_length);
        let expected = Ok((0, Vector2::new(0.5, -0.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // full path length of first seg
    {
        let target_path_length = seg_length(pline[0], pline[1]);
        let r = slice.find_point_at_path_length(&pline, target_path_length);
        let expected = Ok((0, Vector2::new(1.0, 0.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // half path length into second seg
    {
        let target_path_length =
            seg_length(pline[0], pline[1]) + seg_length(pline[1], pline[2]) / 2.0;
        let r = slice.find_point_at_path_length(&pline, target_path_length);
        let expected = Ok((1, Vector2::new(0.5, 0.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // half path length into third seg
    {
        let target_path_length = seg_length(pline[0], pline[1])
            + seg_length(pline[1], pline[2])
            + seg_length(pline[2], pline[3]) / 2.0;
        let r = slice.find_point_at_path_length(&pline, target_path_length);
        let expected = Ok((2, Vector2::new(1.0, 1.5)));
        assert_path_length_result_eq!(r, expected);
    }

    // sub slice tests (mostly to validate segment index offset)
    let sub_slice =
        OpenPlineSlice::from_slice_points(&pline, pline[2].pos(), 2, pline[3].pos(), 3, POS_EQ_EPS)
            .unwrap();
    let sub_slice_length = seg_length(pline[2], pline[3]);

    // 0 path length (point at very start)
    {
        let r = sub_slice.find_point_at_path_length(&pline, 0.0);
        let expected = Ok((0, Vector2::new(1.0, 1.0)));
        assert_path_length_result_eq!(r, expected);
    }

    // total path length (point at very end)
    {
        let r = sub_slice.find_point_at_path_length(&pline, sub_slice_length);
        let expected = Ok((0, Vector2::new(1.0, 2.0)));
        assert_path_length_result_eq!(r, expected);
    }
}
