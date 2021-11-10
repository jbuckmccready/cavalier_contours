use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, PI};

use cavalier_contours::{
    assert_fuzzy_eq,
    core::math::{bulge_from_angle, point_on_circle, Vector2},
    pline_closed, pline_open,
    polyline::{PlineCreation, PlineSource, PlineSourceMut, PlineVertex, PlineViewData, Polyline},
};

const POS_EQ_EPS: f64 = 1e-5;

#[test]
fn from_slice_points_single_seg() {
    let pline = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];

    // complete polyline
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // complete polyline (end segment index on top of final vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // slice from start to middle
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end (end segment index on top of final vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from first third to second third of the segment
    {
        let start_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + FRAC_PI_3);
        let end_point = point_on_circle(0.5, Vector2::new(0.5, 0.0), PI + 2.0 * FRAC_PI_3);
        let slice =
            PlineViewData::from_slice_points(&pline, start_point, 0, end_point, 0, POS_EQ_EPS)
                .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
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
        let slice = PlineViewData::from_slice_points(
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
        let slice = PlineViewData::from_slice_points(
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
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // complete polyline (end segment index on top of last vertex)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 1.0),
            2,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        assert_fuzzy_eq!(&pline_from_slice, &pline);
    }

    // slice from start to middle of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.0, 0.0, bulge), (0.5, -0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle to end of first segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to second vertex
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.0),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from start to middle of second segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            1,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from second vertex to middle of second segment (using previous index for start)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.0),
            0,
            Vector2::new(1.0, 0.5),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let expected_result = pline_open![(1.0, 0.0, 0.0), (1.0, 0.5, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of first segment to last vertex position
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(0.5, -0.5),
            0,
            Vector2::new(1.0, 1.0),
            1,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
        let bulge = bulge_from_angle(FRAC_PI_2);
        let expected_result = pline_open![(0.5, -0.5, bulge), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)];
        assert_fuzzy_eq!(&pline_from_slice, &expected_result);
    }

    // slice from middle of end segment to middle of first segment (wrapping)
    {
        let slice = PlineViewData::from_slice_points(
            &pline,
            Vector2::new(1.0, 0.5),
            1,
            Vector2::new(0.5, -0.5),
            0,
            POS_EQ_EPS,
        )
        .unwrap();

        let pline_from_slice = Polyline::create_from(&slice.view(&pline));
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
        let slice = PlineViewData::from_slice_points(
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
        let slice = PlineViewData::from_slice_points(
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
