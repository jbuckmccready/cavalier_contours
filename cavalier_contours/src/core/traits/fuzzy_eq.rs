/// Trait for fuzzy equality comparisons with floating point numbers.
///
/// This trait provides methods for comparing floating point values with a tolerance
/// (epsilon) to account for floating point precision issues. It's essential for
/// geometric computations where exact equality is rarely achievable due to
/// floating point arithmetic limitations.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::traits::*;
/// let a = 0.1 + 0.2;
/// let b = 0.3;
///
/// // Direct comparison would fail due to floating point precision
/// assert_ne!(a, b);
///
/// // Fuzzy comparison succeeds
/// assert!(a.fuzzy_eq(b));
/// ```
pub trait FuzzyEq: Sized + Copy {
    /// Returns the default epsilon value for fuzzy comparisons.
    fn fuzzy_epsilon() -> Self;

    /// Returns `true` is this object is approximately equal to the other one, using
    /// a provided epsilon value.
    fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: Self) -> bool;

    /// Returns `true` is this object is approximately equal to the other one, using
    /// the implemented [FuzzyEq::fuzzy_epsilon] value.
    #[inline]
    fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, Self::fuzzy_epsilon())
    }

    /// Returns `true` if this value is approximately equal to zero, using
    /// a provided epsilon value.
    fn fuzzy_eq_zero_eps(&self, fuzzy_epsilon: Self) -> bool;

    /// Returns `true` if this value is approximately equal to zero, using
    /// the implemented [FuzzyEq::fuzzy_epsilon] value.
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
