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
