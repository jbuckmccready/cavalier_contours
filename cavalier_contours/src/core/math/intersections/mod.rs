mod circle_circle;
mod line_circle;
mod line_line;

use super::Vector2;
use crate::core::traits::Real;

/// An infinite line used by whole-curve intersection math.
#[derive(Debug, Copy, Clone)]
pub struct Line<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub direction: Vector2<T>,
}

/// A full circle used by whole-curve intersection math.
#[derive(Debug, Copy, Clone)]
pub struct Circle<T>
where
    T: Real,
{
    pub center: Vector2<T>,
    pub radius: T,
}

/// One point produced by a whole-curve intersection calculation.
#[derive(Debug, Copy, Clone)]
pub struct IntersectionPoint<T>
where
    T: Real,
{
    pub point: Vector2<T>,
    pub error_squared: T,
}

impl<T> IntersectionPoint<T>
where
    T: Real,
{
    #[inline]
    pub(super) fn exact(point: Vector2<T>) -> Self {
        Self {
            point,
            error_squared: T::zero(),
        }
    }

    #[inline]
    pub(super) fn adjusted(point: Vector2<T>, error_squared: T) -> Self {
        Self {
            point,
            error_squared,
        }
    }
}

/// Result of intersecting two whole lines or circles.
#[derive(Debug, Copy, Clone)]
pub enum CurveIntersections<T>
where
    T: Real,
{
    None,
    Tangent(IntersectionPoint<T>),
    One(IntersectionPoint<T>),
    Two([IntersectionPoint<T>; 2]),
    Coincident,
}

pub use circle_circle::intersect_circles;
pub use line_circle::intersect_line_circle;
pub use line_line::intersect_lines;
