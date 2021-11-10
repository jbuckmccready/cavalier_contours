mod test_utils;

use cavalier_contours::polyline::{
    BooleanOp, BooleanPlineSlice, BooleanResult, BooleanResultPline, PlineCreation, PlineSource,
    PlineSourceMut, Polyline,
};
use test_utils::{
    create_property_set, property_sets_match, property_sets_match_abs_a, ModifiedPlineSet,
    ModifiedPlineSetVisitor, ModifiedPlineState, PlineProperties,
};

fn create_boolean_property_set(polylines: &[BooleanResultPline<Polyline>]) -> Vec<PlineProperties> {
    for r in polylines {
        assert!(
            r.pline
                .remove_repeat_pos(PlineProperties::POS_EQ_EPS)
                .is_none(),
            "boolean result should not have repeat positioned vertexes"
        );
    }
    create_property_set(polylines.iter().map(|p| &p.pline), false)
}

fn run_same_boolean_test(
    self1: &Polyline<f64>,
    self2: &Polyline<f64>,
    self1_state: &ModifiedPlineState,
    self2_state: &ModifiedPlineState,
    input_properties: &PlineProperties,
) {
    use cavalier_contours::polyline::BooleanOp::*;
    // test same polyline
    for &op in [Or, And].iter() {
        let result = self1.boolean(self2, op);
        let mut passed = result.pos_plines.len() == 1 && result.neg_plines.is_empty();
        if passed {
            let result_properties = PlineProperties::from_pline(
                &result.pos_plines[0].pline,
                self2_state.inverted_direction,
            );
            passed = property_sets_match(&[result_properties], &[*input_properties]);
        }

        assert!(
            passed,
            "boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    for &op in [Not, Xor].iter() {
        let result = self1.boolean(self2, op);
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // test same polyline disjoint by translating
    let extents = self1.extents().unwrap();
    let disjoint1 = {
        let mut c = self1.clone();
        c.translate_mut(1.0 + extents.max_x - extents.min_x, 0.0);
        c
    };

    let disjoint1_properties = PlineProperties::from_pline(&disjoint1, false);

    // disjoint OR
    {
        let op = Or;
        let expected = &[disjoint1_properties, *input_properties];
        let result = disjoint1.boolean(self2, op);
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // disjoint AND
    {
        let op = And;
        let result = disjoint1.boolean(self2, op);
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // disjoint NOT
    {
        let op = Not;
        let expected = &[disjoint1_properties];
        let result = disjoint1.boolean(self2, op);
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // disjoint XOR
    {
        let op = Xor;
        let expected = &[disjoint1_properties, *input_properties];
        let result = disjoint1.boolean(self2, op);
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "disjoint test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // test same polyline but offset one of them to be fully enclosed by the other
    let offset = if self1.area() < 0.0 { -0.2 } else { 0.2 };
    let self1_inward_offset = self1.parallel_offset(offset).remove(0);

    let offset_properties = &[PlineProperties::from_pline(&self1_inward_offset, false)];

    // enclosed OR
    {
        let op = Or;
        let expected = &[*input_properties];
        let result = self2.boolean(&self1_inward_offset, op);
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // enclosed AND
    {
        let op = And;
        let expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        let result_properties = create_boolean_property_set(&result.pos_plines);
        let passed =
            property_sets_match_abs_a(&result_properties, expected) && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // enclosed self2 NOT self1_offset
    {
        let op = Not;
        let pos_expected = &[*input_properties];
        let neg_expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        let pos_pline_result_properties = create_boolean_property_set(&result.pos_plines);
        let neg_pline_result_properties = create_boolean_property_set(&result.neg_plines);
        let passed = property_sets_match_abs_a(&pos_pline_result_properties, pos_expected)
            && property_sets_match_abs_a(&neg_pline_result_properties, neg_expected);
        assert!(
            passed,
            "enclosed test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // enclosed self1_offset NOT self2
    {
        let op = Not;
        let result = self1_inward_offset.boolean(self2, op);
        let passed = result.pos_plines.is_empty() && result.neg_plines.is_empty();
        assert!(
            passed,
            "enclosed test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }

    // enclosed XOR
    {
        let op = Xor;
        let pos_expected = &[*input_properties];
        let neg_expected = offset_properties;
        let result = self2.boolean(&self1_inward_offset, op);
        let pos_pline_result_properties = create_boolean_property_set(&result.pos_plines);
        let neg_pline_result_properties = create_boolean_property_set(&result.neg_plines);
        let passed = property_sets_match_abs_a(&pos_pline_result_properties, pos_expected)
            && property_sets_match_abs_a(&neg_pline_result_properties, neg_expected);
        assert!(
            passed,
            "enclosed test failed, boolean op: {:?}, modified state1: {:?}, modified state2: {:?}",
            op, self1_state, self2_state
        );
    }
}

struct SameBooleanTestVisitor<'a> {
    input_properties: &'a PlineProperties,
    other_set: ModifiedPlineSet<'a>,
}

impl<'a> ModifiedPlineSetVisitor for SameBooleanTestVisitor<'a> {
    fn visit(&mut self, modified_pline: Polyline<f64>, pline_state: ModifiedPlineState) {
        // test every combination of direction and index position cycle
        self.other_set
            .accept_closure(&mut |modified_pline2, pline_state2| {
                run_same_boolean_test(
                    &modified_pline,
                    &modified_pline2,
                    &pline_state,
                    &pline_state2,
                    self.input_properties,
                )
            });
    }
}

fn run_same_boolean_tests(input: &Polyline<f64>) {
    let pline_properties = PlineProperties::from_pline(input, false);
    let other_modified_set = ModifiedPlineSet::new(input, true, true);
    let mut visitor = SameBooleanTestVisitor {
        input_properties: &pline_properties,
        other_set: other_modified_set,
    };

    let test_set = ModifiedPlineSet::new(input, true, true);

    test_set.accept(&mut visitor);
}

macro_rules! declare_same_boolean_tests {
    ($($name:ident { $($pline:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_same_boolean_tests(&$pline);
                )+
            }
        )+
    };
}

mod test_same {
    use super::*;
    use cavalier_contours::pline_closed;

    declare_same_boolean_tests!(
        origin_circle {
            pline_closed![ (-1.0, 0.0, 1.0), (1.0, 0.0, 1.0) ]
        }
        origin_circle2 {
            pline_closed![ (0.0, -1.0, 1.0), (0.0, 1.0, 1.0) ]
        }
        rectangle {
            pline_closed![ (0.0, 0.0, 0.0), (20.0, 0.0, 0.0), (20.0, 10.0, 0.0), (0.0, 10.0, 0.0) ]
        }
        diamond {
            pline_closed![ (-10.0, 0.0, 0.0), (0.0, 10.0, 0.0), (10.0, 0.0, 0.0), (0.0, -10.0, 0.0) ]
        }
        case1 {
            pline_closed![(27.804688, 1.0, 0.0),
                           (28.46842055794889, 0.3429054695163245, 0.0),
                           (32.34577133994935, 0.9269762697003898, 0.0),
                           (32.38116957207762, 1.451312562563487, 0.0),
                           (31.5, 1.0, -0.31783751349740424),
                           (30.79289310940682, 1.5, 0.0),
                           (29.20710689059337, 1.5, -0.31783754777018053),
                           (28.49999981323106, 1.00000000000007, 0.0)]
        }
        case2 {
            pline_closed![(27.804688, 1.0, 0.0),
                           (27.804688, 0.75, 0.0),
                           (32.195313, 0.75, 0.0),
                           (32.195313, 1.0, 0.0),
                           (31.5, 1.0, -0.3178375134974),
                           (30.792893109407, 1.5, 0.0),
                           (29.207106890593, 1.5, -0.31783754777018),
                           (28.499999813231, 1.0000000000001, 0.0)]
        }
        case3 {
            pline_closed![ (0.0, 0.0, 0.0), (120.0, 0.0, 0.0), (120.0, 40.0, 0.0), (0.0, 40.0, 0.0) ]
        }
    );
}

fn verify_slice_set(
    result_pline: &BooleanResultPline<Polyline>,
    pline1: &Polyline,
    pline2: &Polyline,
) {
    if result_pline.subslices.is_empty() {
        return;
    }

    let slice_to_pline = |s: &BooleanPlineSlice| {
        let source = if s.source_is_pline1 { pline1 } else { pline2 };
        Polyline::create_from_remove_repeat(&s.view(source), PlineProperties::POS_EQ_EPS)
    };

    let stitch_slice_onto = |s: &BooleanPlineSlice, target: &mut Polyline| {
        let source = if s.source_is_pline1 { pline1 } else { pline2 };
        target.extend_remove_repeat(&s.view(source), PlineProperties::POS_EQ_EPS)
    };

    let first_slice = &result_pline.subslices[0];
    let mut stitched = slice_to_pline(first_slice);

    for s in result_pline.subslices.iter().skip(1) {
        stitched.remove_last();
        stitch_slice_onto(s, &mut stitched);
    }
    assert!(
        stitched[0].pos().fuzzy_eq(stitched.last().unwrap().pos()),
        "start does not connect with end when stitching slices together"
    );
    stitched.remove_last();
    stitched.set_is_closed(true);

    let expected_properties = PlineProperties::from_pline(&result_pline.pline, false);
    let stitched_properties = PlineProperties::from_pline(&stitched, false);

    assert!(
        expected_properties.fuzzy_eq_eps(&stitched_properties, PlineProperties::PROP_CMP_EPS),
        "slices stitched together do not match result polyline, expected: {:?}, actual: {:?}",
        expected_properties,
        stitched_properties
    );
}

fn verify_all_slices(
    pline1: &Polyline,
    pline2: &Polyline,
    boolean_result: &BooleanResult<Polyline>,
) {
    for result_pline in boolean_result
        .pos_plines
        .iter()
        .chain(boolean_result.neg_plines.iter())
    {
        verify_slice_set(result_pline, pline1, pline2);
    }
}

fn run_pline_boolean_tests(
    pline1: &Polyline<f64>,
    pline2: &Polyline<f64>,
    cases: &[(BooleanOp, &[PlineProperties], &[PlineProperties])],
) {
    let test_set1 = ModifiedPlineSet::new(pline1, true, true);
    let test_set2 = ModifiedPlineSet::new(pline2, true, true);

    // test every combination of direction and index position cycle between the two polylines
    test_set1.accept_closure(&mut |modified_pline1, state1| {
        test_set2.accept_closure(&mut |modified_pline2, state2| {
            for &(op, pos_set_expected, neg_set_expected) in cases {
                let result = modified_pline1.boolean(&modified_pline2, op);
                let pos_set_result = create_boolean_property_set(&result.pos_plines);
                let neg_set_result = create_boolean_property_set(&result.neg_plines);
                let passed = property_sets_match_abs_a(&pos_set_result, &pos_set_expected)
                    && property_sets_match_abs_a(&neg_set_result, &neg_set_expected);
                assert!(
                    passed,
                    "property sets do not match\nop: {:?}\nstate1: {:?}\nstate2: {:?}",
                    op, state1, state2
                );

                println!("{:?}", op);
                verify_all_slices(&modified_pline1, &modified_pline2, &result);
            }
        });
    });
}

macro_rules! declare_boolean_tests {
    ($($name:ident { $($value:expr => $expected:expr),+ $(,)? })*) => {
        $(
            #[test]
            fn $name() {
                $(
                    run_pline_boolean_tests(&$value.0, &$value.1, &$expected);
                )+
            }
        )+
    };
}

mod test_simple {
    use cavalier_contours::pline_closed;

    use super::*;
    declare_boolean_tests! {
        rectangle_slicing_circle {
            (
                pline_closed![(0.0, 1.0, 1.0),(10.0, 1.0, 1.0)],
                pline_closed![(3.0, -10.0, 0.0),(6.0, -10.0, 0.0),(6.0, 10.0, 0.0),(3.0, 10.0, 0.0)])
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(8, 109.15381629282, 52.324068506275, 0.0, -10.0, 10.0, 10.0)], &[]),
                (BooleanOp::Not, &[PlineProperties::new(2, 29.336980664548, 23.492343031178, 6.0, -3.8989794855664, 10.0, 5.8989794855664), PlineProperties::new(2, 19.816835628274, 20.757946197186, 0.0, -3.5825756949558, 3.0, 5.5825756949558)], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 29.386000046923, 25.091858029623, 3.0, -4.0, 6.0, 6.0)], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(2, 29.336980664548, 23.492343031178, 6.0, -3.8989794855664, 10.0, 5.8989794855664), PlineProperties::new(2, 19.816835628274, 20.757946197186, -8.8817841970013e-16, -3.5825756949558, 3.0, 5.5825756949558), PlineProperties::new(4, -18.306999976538, 18.582818653767, 3.0, -10.0, 6.0, -3.5825756949558), PlineProperties::new(4, -12.306999976538, 14.582818653767, 3.0, 5.5825756949558, 6.0, 10.0), ], &[])
            ]
        }
        rectangle_over_half_of_circle {
            (
                pline_closed![(-50.0, 0.0, 1.0), (50.0, 0.0, 1.0)],
                pline_closed![(-50.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (-50.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 8926.990816987241, 357.0796326794897, -50.0, -50.0, 50.0, 50.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(2, 3926.9908169872415, 257.0796326794897, -50.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::Not, &[PlineProperties::new(2, -3926.9908169872415, 257.0796326794897, -50.0, -50.0, 50.0, 0.0)], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(2, -3926.9908169872415, 257.0796326794897, -50.0, -50.0, 50.0, 0.0), PlineProperties::new(3, 536.504591506379, 178.53981633974485, 0.0, 0.0, 50.0, 50.0), PlineProperties::new(3, 536.504591506379, 178.53981633974485, -50.0, 0.0, 0.0, 50.0)], &[])
            ]
        }
        rectangle_in_rectangle_one_edge_overlap {
            (
                pline_closed![(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)],
                pline_closed![(10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 40.0, 0.0), (10.0, 40.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1200.0, 140.0, 10.0, 10.0, 50.0, 40.0)], &[]),
                (BooleanOp::Not, &[PlineProperties::new(8, -1300.0, 280.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(8, -1300.0, 280.0, 0.0, 0.0, 50.0, 50.0)], &[])
            ]
        }
        rectangle_in_rectangle_one_edge_overlap_flipped_order {
            (
                pline_closed![(10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 40.0, 0.0), (10.0, 40.0, 0.0)],
                pline_closed![(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1200.0, 140.0, 10.0, 10.0, 50.0, 40.0)], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(8, 1300.0, 280.0, 0.0, 0.0, 50.0, 50.0)], &[])
            ]
        }
        rectangle_in_rectangle_two_edge_overlap {
            (
                pline_closed![(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)],
                pline_closed![(10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 50.0, 0.0), (10.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1600.0, 160.0, 10.0, 10.0, 50.0, 50.0)], &[]),
                (BooleanOp::Not, &[PlineProperties::new(6, -900.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(6, -900.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[])
            ]
        }
        rectangle_in_rectangle_two_edge_overlap_flipped_order {
            (
                pline_closed![(10.0, 10.0, 0.0), (50.0, 10.0, 0.0), (50.0, 50.0, 0.0), (10.0, 50.0, 0.0)],
                pline_closed![(0.0, 0.0, 0.0), (50.0, 0.0, 0.0), (50.0, 50.0, 0.0), (0.0, 50.0, 0.0)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(4, 2500.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(4, 1600.0, 160.0, 10.0, 10.0, 50.0, 50.0)], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(6, 900.0, 200.0, 0.0, 0.0, 50.0, 50.0)], &[])
            ]
        }
    }
}

mod test_specific {
    use cavalier_contours::pline_closed;

    use super::*;
    declare_boolean_tests! {
        mostly_overlapping_case1 {
            (
                pline_closed![(100.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
                pline_closed![(30.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(8, 49966.081474155624, 1146.9875028214367, 30.0, -45.0, 404.41586988912127, 210.0)], &[]),
                (BooleanOp::And, &[PlineProperties::new(8, 44023.79002726299, 1033.0222984555098, 80.0, -45.0, 404.41586988912127, 210.0)], &[]),
                (BooleanOp::Not, &[], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(4, 5942.291446892632, 585.653349720364, 30.0, 88.52457514062631, 280.0, 172.99920254980566)], &[])
            ]
        }
        mostly_overlapping_case2 {
            (
                pline_closed![(100.0, 100.0, -0.5), (30.0, 100.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)],
                pline_closed![(30.0, 100.0, -0.5), (80.0, 90.0, 0.374794619217547), (210.0, 0.0, 0.0), (230.0, 0.0, 1.0), (320.0, 0.0, -0.5), (280.0, 0.0, 0.5), (390.0, 210.0, 0.0), (280.0, 120.0, 0.5)]
            )
            =>
            [
                (BooleanOp::Or, &[PlineProperties::new(7, 54254.07433137387, 1140.0400125191409, 29.99999999999998, -45.0, 404.41586988912127, 210.0)], &[PlineProperties::new(3, -884.615153413562, 120.820138445405, 29.99999999999998, 82.5, 81.20071955870648, 108.1188719599549)]),
                (BooleanOp::And, &[PlineProperties::new(8, 44035.3323189534, 1026.5255074451134, 81.20071955870648, -45.0, 404.41586988912127, 210.0)], &[]),
                (BooleanOp::Not, &[PlineProperties::new(3, -3403.377703804679, 449.8260354415685, 29.99999999999998, -3.4440267329434846, 210.0, 99.99999999999999)], &[]),
                (BooleanOp::Xor, &[PlineProperties::new(3, -3403.377703804679, 449.8260354415685, 29.99999999999998, -3.4440267329434846, 210.0, 99.99999999999999), PlineProperties::new(5, 5930.749155202231, 588.2593437968856, 29.99999999999998, 85.61012812790688, 280.0, 172.99920254980566)], &[])
            ]
        }
    }
}
