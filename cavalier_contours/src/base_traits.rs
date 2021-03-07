use static_aabb2d_index::IndexableNum;

pub trait Real:
    num_traits::real::Real
    + num_traits::Bounded
    + FuzzyOrd
    + std::default::Default
    + std::fmt::Debug
    + IndexableNum
{
    #[inline]
    fn pi() -> Self {
        Self::from(std::f64::consts::PI).unwrap()
    }

    #[inline]
    fn tau() -> Self {
        Self::from(std::f64::consts::TAU).unwrap()
    }

    #[inline]
    fn two() -> Self {
        Self::one() + Self::one()
    }

    #[inline]
    fn four() -> Self {
        Self::two() + Self::two()
    }

    #[inline]
    fn min_value() -> Self {
        num_traits::real::Real::min_value()
    }

    #[inline]
    fn max_value() -> Self {
        num_traits::real::Real::max_value()
    }
}

impl Real for f32 {
    #[inline]
    fn pi() -> Self {
        std::f32::consts::PI
    }

    #[inline]
    fn tau() -> Self {
        std::f32::consts::TAU
    }

    #[inline]
    fn two() -> Self {
        2.0f32
    }

    #[inline]
    fn four() -> Self {
        4.0f32
    }
}

impl Real for f64 {
    #[inline]
    fn pi() -> Self {
        std::f64::consts::PI
    }

    #[inline]
    fn tau() -> Self {
        std::f64::consts::TAU
    }

    #[inline]
    fn two() -> Self {
        2.0f64
    }

    #[inline]
    fn four() -> Self {
        4.0f64
    }
}

pub trait FuzzyEq: Sized + Copy {
    fn fuzzy_epsilon() -> Self;

    /// Returns `true` is this object is approximately equal to the other one, using
    /// a provided epsilon value.
    fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;

    /// Returns `true` is this object is approximately equal to the other one, using
    /// the implemented `ABS_EPSILON` epsilon value.
    #[inline]
    fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, Self::fuzzy_epsilon())
    }

    fn fuzzy_eq_zero_eps(&self, fuzzy_epsilon: Self) -> bool;

    #[inline]
    fn fuzzy_eq_zero(&self) -> bool {
        self.fuzzy_eq_zero_eps(Self::fuzzy_epsilon())
    }
}

macro_rules! impl_fuzzy_eq {
    ($ty:ty, $eps:expr) => {
        impl FuzzyEq for $ty {
            #[inline]
            fn fuzzy_epsilon() -> Self {
                $eps
            }
            #[inline]
            fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool {
                (*self - other).abs() < fuzzy_epsilon
            }
            #[inline]
            fn fuzzy_eq_zero_eps(&self, fuzzy_epsilon: Self) -> bool {
                self.abs() < fuzzy_epsilon
            }
        }
    };
}

impl_fuzzy_eq!(f32, 1.0e-8);
impl_fuzzy_eq!(f64, 1.0e-8);

pub trait FuzzyOrd: FuzzyEq {
    fn fuzzy_gt_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;
    /// Fuzzy greater than.
    #[inline]
    fn fuzzy_gt(&self, other: Self) -> bool {
        self.fuzzy_gt_eps(other, Self::fuzzy_epsilon())
    }

    fn fuzzy_lt_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;
    /// Fuzzy less than.
    #[inline]
    fn fuzzy_lt(&self, other: Self) -> bool {
        self.fuzzy_lt_eps(other, Self::fuzzy_epsilon())
    }

    /// Test if `self` is in range between `min` and `max` with some epsilon for fuzzy comparing.
    ///
    /// See [FuzzyOrd::fuzzy_in_range] function to use default fuzzy epsilon.
    ///
    /// # Examples
    ///
    /// ```
    /// # use cavalier_contours::*;
    /// assert!(0.99f64.fuzzy_in_range_eps(1.0, 2.0, 0.05));
    /// assert!(1.5f64.fuzzy_in_range_eps(1.0, 2.0, 1e-5));
    /// assert!(2.0f64.fuzzy_in_range_eps(1.0, 2.0, 1e-5));
    ///```
    #[inline]
    fn fuzzy_in_range_eps(&self, min: Self, max: Self, fuzzy_epsilon: Self) -> bool {
        self.fuzzy_gt_eps(min, fuzzy_epsilon) && self.fuzzy_lt_eps(max, fuzzy_epsilon)
    }

    /// Same as [FuzzyOrd::fuzzy_in_range_eps] using a default epsilon.
    ///
    /// Default epsilon is [fuzzy_epsilon](crate::base_traits::FuzzyEq::fuzzy_epsilon)
    /// from [FuzzyEq](crate::base_traits::FuzzyEq) trait.
    #[inline]
    fn fuzzy_in_range(&self, min: Self, max: Self) -> bool {
        self.fuzzy_in_range_eps(min, max, Self::fuzzy_epsilon())
    }
}

macro_rules! impl_fuzzy_ord {
    ($ty:ty) => {
        impl FuzzyOrd for $ty {
            #[inline]
            fn fuzzy_gt_eps(&self, other: $ty, fuzzy_epsilon: $ty) -> bool {
                self + fuzzy_epsilon > other
            }
            #[inline]
            fn fuzzy_lt_eps(&self, other: $ty, fuzzy_epsilon: $ty) -> bool {
                *self < other + fuzzy_epsilon
            }
        }
    };
}

impl_fuzzy_ord!(f32);
impl_fuzzy_ord!(f64);
