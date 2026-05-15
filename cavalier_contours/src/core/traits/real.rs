use super::FuzzyOrd;
use static_aabb2d_index::IndexableNum;

/// Trait representing a real number (e.g. 1.1, -3.5, etc.) that can be fuzzy compared and ordered.
pub trait Real:
    num_traits::real::Real
    + num_traits::Bounded
    + FuzzyOrd
    + std::default::Default
    + std::fmt::Debug
    + IndexableNum
    + 'static
{
    /// Return pi for the concrete floating point type.
    #[inline]
    fn pi() -> Self {
        Self::from(std::f64::consts::PI).unwrap()
    }

    /// Return tau, equal to `2 * pi`, for the concrete floating point type.
    #[inline]
    fn tau() -> Self {
        Self::from(std::f64::consts::TAU).unwrap()
    }

    /// Return the numeric value `2`.
    #[inline]
    fn two() -> Self {
        Self::one() + Self::one()
    }

    /// Return the numeric value `4`.
    #[inline]
    fn four() -> Self {
        Self::two() + Self::two()
    }

    /// Return the smallest finite value representable by the type.
    #[inline]
    fn min_value() -> Self {
        num_traits::real::Real::min_value()
    }

    /// Return the largest finite value representable by the type.
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
