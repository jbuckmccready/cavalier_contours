use super::FuzzyEq;

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
    /// # use cavalier_contours::core::traits::*;
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
    /// Default epsilon is [fuzzy_epsilon](crate::core::traits::FuzzyEq::fuzzy_epsilon)
    /// from [FuzzyEq](crate::core::traits::FuzzyEq) trait.
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
