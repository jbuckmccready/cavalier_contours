use crate::{base_math::parametric_from_point, Real, Vector2};

/// Holds the result of finding the intersect between two line segments.
#[derive(Debug, Copy, Clone)]
pub enum LineLineIntr<T>
where
    T: Real,
{
    /// No intersect, segments are parallel and not collinear.
    NoIntersect,
    /// There is a true intersect between the line segments.
    TrueIntersect {
        /// Parametric value for intersect on first segment.
        seg1_t: T,
        /// Parametric value for intersect on second segment.
        seg2_t: T,
    },
    /// Segments overlap each other (are collinear) by some amount.
    Overlapping {
        /// Parametric value for start of coincidence along second segment.
        seg2_t0: T,
        /// Parametric value for end of coincidence along second segment.
        seg2_t1: T,
    },
    /// There is an intersect between the lines but one or both of the segments must be extended.
    FalseIntersect {
        /// Parametric value for intersect on first segment.
        seg1_t: T,
        /// Parametric value for intersect on second segment.
        seg2_t: T,
    },
}

/// Finds the intersects between two lines segments.
///
/// Line segments are defined by `v1->v2` and `u1->u2`.
/// Handles the cases where the lines may be parallel, collinear, or single points.
pub fn line_line_intr<T>(
    v1: Vector2<T>,
    v2: Vector2<T>,
    u1: Vector2<T>,
    u2: Vector2<T>,
) -> LineLineIntr<T>
where
    T: Real,
{
    // This implementation works by processing the segments in parametric equation form and using
    // perpendicular products
    // http://geomalgorithms.com/a05-_intersect-1.html
    // http://mathworld.wolfram.com/PerpDotProduct.html

    use LineLineIntr::*;

    let v = v2 - v1;
    let u = u2 - u1;
    let d = v.perp_dot(u);
    let w = v1 - u1;

    // threshold check here to avoid almost parallel lines resulting in very distant intersection
    if !d.fuzzy_eq_zero() {
        // segments not parallel or collinear
        let seg1_t = u.perp_dot(w) / d;
        let seg2_t = v.perp_dot(w) / d;
        if !seg1_t.fuzzy_in_range(T::zero(), T::one())
            || !seg2_t.fuzzy_in_range(T::zero(), T::one())
        {
            return FalseIntersect { seg1_t, seg2_t };
        }
        return TrueIntersect { seg1_t, seg2_t };
    }

    // segments are parallel and possibly collinear
    let a = v.perp_dot(w);
    let b = u.perp_dot(w);

    // threshold check here, we consider almost parallel lines to be parallel
    if !a.fuzzy_eq_zero() || !b.fuzzy_eq_zero() {
        // parallel and not collinear so no intersect
        return NoIntersect;
    }

    // either collinear or degenerate (segments are single points)
    let v_is_point = v1.fuzzy_eq(v2);
    let u_is_point = u1.fuzzy_eq(u2);

    if v_is_point && u_is_point {
        // both segments are points
        if v1.fuzzy_eq(u1) {
            // same point
            return TrueIntersect {
                seg1_t: T::zero(),
                seg2_t: T::zero(),
            };
        }
        // distinct points
        return NoIntersect;
    }

    if v_is_point {
        // v is point and v is not a point
        let seg2_t = parametric_from_point(u1, u2, v1);
        if seg2_t.fuzzy_in_range(T::zero(), T::one()) {
            return TrueIntersect {
                seg1_t: T::zero(),
                seg2_t,
            };
        }

        return NoIntersect;
    }

    if u_is_point {
        // u is point and u is not a point
        let seg1_t = parametric_from_point(v1, v2, u1);
        if seg1_t.fuzzy_in_range(T::zero(), T::one()) {
            return TrueIntersect {
                seg1_t,
                seg2_t: T::zero(),
            };
        }

        return NoIntersect;
    }

    // neither segment is a point, check if they overlap
    let w2 = v2 - u1;
    let (mut seg2_t0, mut seg2_t1) = if u.x.fuzzy_eq_zero() {
        (w.y / u.y, w2.y / u.y)
    } else {
        (w.x / u.x, w2.x / u.x)
    };

    if seg2_t0 > seg2_t1 {
        std::mem::swap(&mut seg2_t0, &mut seg2_t1);
    }

    // using threshold check here to make intersect "sticky" to prefer considering it an intersect
    if !seg2_t0.fuzzy_lt(T::one()) || !seg2_t1.fuzzy_gt(T::zero()) {
        return NoIntersect;
    }

    seg2_t0 = num_traits::real::Real::max(seg2_t0, T::zero());
    seg2_t1 = num_traits::real::Real::min(seg2_t1, T::one());

    if (seg2_t1 - seg2_t0).fuzzy_eq_zero() {
        // intersect is a single point (segments line up end to end)
        // determine if seg1_t is 0.0 or 1.0 (will not match seg2_t since they only touch at ends)
        let seg1_t = if !seg2_t0.fuzzy_eq_zero() {
            T::zero()
        } else {
            T::one()
        };
        return TrueIntersect {
            seg1_t,
            seg2_t: seg2_t0,
        };
    }

    Overlapping { seg2_t0, seg2_t1 }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_3, FRAC_PI_4, FRAC_PI_6, FRAC_PI_8};

    use super::*;
    use crate::base_traits::FuzzyEq;
    use LineLineIntr::*;

    macro_rules! assert_case_eq {
        ($left:expr, $right:expr) => {
            match ($left, $right) {
                (NoIntersect, NoIntersect) => {}
                (
                    TrueIntersect {
                        seg1_t: a1,
                        seg2_t: b1,
                    },
                    TrueIntersect {
                        seg1_t: a2,
                        seg2_t: b2,
                    },
                )
                | (
                    FalseIntersect {
                        seg1_t: a1,
                        seg2_t: b1,
                    },
                    FalseIntersect {
                        seg1_t: a2,
                        seg2_t: b2,
                    },
                )
                | (
                    Overlapping {
                        seg2_t0: a1,
                        seg2_t1: b1,
                    },
                    Overlapping {
                        seg2_t0: a2,
                        seg2_t1: b2,
                    },
                ) if a1.fuzzy_eq(a2) && b1.fuzzy_eq(b2) => {}
                _ => panic!(
                    "intersect cases do not match: left: {:?}, right: {:?}",
                    $left, $right
                ),
            };
        };
    }

    const TEST_ROTATION_ANGLES: &[f64] = &[FRAC_PI_8, FRAC_PI_6, FRAC_PI_4, FRAC_PI_3, FRAC_PI_2];

    #[test]
    fn true_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(-1.0, 1.0);
        let v2 = Vector2::new(1.0, -1.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn endpoints_true_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(1.0, 1.0);
        let v2 = Vector2::new(2.0, 2.0);

        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 1.0,
                seg2_t: 0.0
            }
        );

        // flip argument order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 1.0
            }
        );

        // rotate v1->v2 should get same result
        for &angle in TEST_ROTATION_ANGLES {
            let v2 = v2.rotate_about(v1, angle);
            let result = line_line_intr(u1, u2, v1, v2);
            assert_case_eq!(
                result,
                TrueIntersect {
                    seg1_t: 1.0,
                    seg2_t: 0.0
                }
            );
        }
    }

    #[test]
    fn false_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(-0.5, -0.5);
        let v1 = Vector2::new(-1.0, 1.0);
        let v2 = Vector2::new(1.0, -1.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            FalseIntersect {
                seg1_t: 2.0,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn no_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(0.0, 1.0);
        let v2 = Vector2::new(1.0, 2.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_vertical() {
        let u1 = Vector2::new(2.0, 0.0);
        let u2 = Vector2::new(2.0, 1.0);
        let v1 = Vector2::new(-1.0, -1.0);
        let v2 = Vector2::new(-1.0, -2.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn no_intersect_horizontal() {
        let u1 = Vector2::new(-2.0, -1.0);
        let u2 = Vector2::new(2.0, -1.0);
        let v1 = Vector2::new(-1.0, 5.0);
        let v2 = Vector2::new(1.0, 5.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(result, NoIntersect::<f64>);
    }

    #[test]
    fn coincident_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(0.0, 0.0);
        let v2 = Vector2::new(0.5, 0.5);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );
    }

    #[test]
    fn point_intersect() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = Vector2::new(0.0, 0.0);
        let v2 = Vector2::new(0.0, 0.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn point_intersect_vertical() {
        let u1 = Vector2::new(0.0, -1.0);
        let u2 = Vector2::new(0.0, 1.0);
        let v1 = Vector2::new(0.0, 0.0);
        let v2 = Vector2::new(0.0, 0.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn point_intersect_horizontal() {
        let u1 = Vector2::new(-1.0, 0.0);
        let u2 = Vector2::new(1.0, 0.0);
        let v1 = Vector2::new(0.0, 0.0);
        let v2 = Vector2::new(0.0, 0.0);
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.5,
                seg2_t: 0.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.5
            }
        );
    }

    #[test]
    fn point_intersect_at_end() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = u1;
        let v2 = u1;
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 0.0
            }
        );

        // other end
        let v1 = u2;
        let v2 = u2;
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 1.0,
                seg2_t: 0.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            TrueIntersect {
                seg1_t: 0.0,
                seg2_t: 1.0
            }
        );
    }

    #[test]
    fn entirely_coincident_same_direction() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = u1;
        let v2 = u2;
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );

        // rotate both lines together
        for &angle in TEST_ROTATION_ANGLES {
            let u2 = u2.rotate_about(u1, angle);
            let v2 = v2.rotate_about(v1, angle);
            let result = line_line_intr(u1, u2, v1, v2);
            assert_case_eq!(
                result,
                Overlapping {
                    seg2_t0: 0.0,
                    seg2_t1: 1.0
                }
            );
        }
    }
    #[test]
    fn entirely_coincident_opposing_direction() {
        let u1 = Vector2::new(-1.0, -1.0);
        let u2 = Vector2::new(1.0, 1.0);
        let v1 = u2;
        let v2 = u1;
        let result = line_line_intr(u1, u2, v1, v2);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );

        // flip arg order
        let result = line_line_intr(v1, v2, u1, u2);
        assert_case_eq!(
            result,
            Overlapping {
                seg2_t0: 0.0,
                seg2_t1: 1.0
            }
        );

        // rotate both lines together
        for &angle in TEST_ROTATION_ANGLES {
            let u2 = u2.rotate_about(u1, angle);
            let v1 = v1.rotate_about(v2, angle);
            let result = line_line_intr(u1, u2, v1, v2);
            assert_case_eq!(
                result,
                Overlapping {
                    seg2_t0: 0.0,
                    seg2_t1: 1.0
                }
            );
        }
    }
}
