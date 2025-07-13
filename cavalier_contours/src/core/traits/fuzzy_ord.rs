use super::FuzzyEq;

/// Trait for fuzzy ordering comparisons with floating point numbers.
///
/// This trait extends [`FuzzyEq`] to provide fuzzy comparison operations for ordering
/// floating point values. Like fuzzy equality, this accounts for floating point
/// precision issues in greater-than and less-than comparisons.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// let a = 0.1 + 0.2;
/// let b = 0.3;
///
/// // Due to floating point precision, a is actually slightly greater than b
/// assert!(!(a <= b));
///
/// // But fuzzy comparison considers them equal
/// assert!(a.fuzzy_lt(b));
/// ```
pub trait FuzzyOrd: FuzzyEq {
    /// Returns `true` if this value is fuzzy greater than the other, using
    /// a provided epsilon value.
    fn fuzzy_gt_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;
    /// Fuzzy greater than.
    #[inline]
    fn fuzzy_gt(&self, other: Self) -> bool {
        self.fuzzy_gt_eps(other, Self::fuzzy_epsilon())
    }

    /// Returns `true` if this value is fuzzy less than the other, using
    /// a provided epsilon value.
    fn fuzzy_lt_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;

    /// Returns `true` if this value is fuzzy less than the other, using
    /// the default epsilon value.
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
    /// from [FuzzyEq] trait.
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
