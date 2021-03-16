pub trait FuzzyEq: Sized + Copy {
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
