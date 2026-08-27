use super::Vector2;
use crate::core::traits::Real;

/// One point produced by an intersection calculation.
#[derive(Debug, Copy, Clone)]
pub struct IntrPoint<T>
where
    T: Real,
{
    /// Returned intersection point.
    pub point: Vector2<T>,
    /// Sum of squared geometric adjustments used to produce `point`.
    ///
    /// Zero means `point` is an unchanged analytic solution.
    pub error_squared: T,
}

impl<T> IntrPoint<T>
where
    T: Real,
{
    #[inline]
    pub(crate) fn exact(point: Vector2<T>) -> Self {
        Self {
            point,
            error_squared: T::zero(),
        }
    }

    #[inline]
    pub(crate) fn adjusted(point: Vector2<T>, error_sq: T) -> Self {
        Self {
            point,
            error_squared: error_sq,
        }
    }
}

/// Position of an intersection candidate relative to a finite curve domain.
#[derive(Debug, Copy, Clone)]
pub(crate) enum IntrDomainPosition<T>
where
    T: Real,
{
    Inside { path_distance: T },
    Snap { point: Vector2<T>, path_distance: T },
    Outside,
}

/// Finite circular-arc domain used to classify supporting-circle candidates.
#[derive(Debug, Copy, Clone)]
pub(crate) struct ArcDomain<T>
where
    T: Real,
{
    center: Vector2<T>,
    start: Vector2<T>,
    end: Vector2<T>,
    radius: T,
    sweep: T,
    is_clockwise: bool,
}

impl<T> ArcDomain<T>
where
    T: Real,
{
    #[inline]
    pub(crate) fn new(
        center: Vector2<T>,
        radius: T,
        start: Vector2<T>,
        end: Vector2<T>,
        sweep: T,
        is_clockwise: bool,
    ) -> Self {
        Self {
            center,
            start,
            end,
            radius,
            sweep,
            is_clockwise,
        }
    }

    #[inline]
    pub(crate) fn sweep(&self) -> T {
        self.sweep
    }

    #[inline]
    pub(crate) fn forward_angle(&self, point: Vector2<T>) -> T {
        let start_vector = self.start - self.center;
        let point_vector = point - self.center;
        let cross = start_vector.perp_dot(point_vector);
        let oriented_cross = if self.is_clockwise { -cross } else { cross };
        let mut result = T::atan2(oriented_cross, start_vector.dot(point_vector));
        if result < T::zero() {
            result = result + T::tau();
        }
        result
    }

    #[inline]
    pub(crate) fn project_to_circle(&self, point: Vector2<T>) -> Vector2<T> {
        let radial = point - self.center;
        self.center + radial.scale(self.radius / radial.length())
    }

    /// Classifies a supporting-circle solution against this finite arc.
    #[inline]
    pub(crate) fn classify(&self, point: Vector2<T>, epsilon_sq: T) -> IntrDomainPosition<T> {
        let point_vector = point - self.center;
        let radial_difference = point_vector.length() - self.radius;
        if radial_difference * radial_difference >= epsilon_sq {
            return IntrDomainPosition::Outside;
        }
        let forward_angle = self.forward_angle(point);

        if forward_angle <= self.sweep {
            return IntrDomainPosition::Inside {
                path_distance: self.radius * forward_angle,
            };
        }

        let before_start = T::tau() - forward_angle;
        let after_end = forward_angle - self.sweep;
        let (overshoot, endpoint, path_distance) = if before_start < after_end {
            (before_start * self.radius, self.start, T::zero())
        } else {
            (after_end * self.radius, self.end, self.radius * self.sweep)
        };

        let endpoint_distance = point - endpoint;
        if overshoot * overshoot < epsilon_sq && endpoint_distance.length_squared() < epsilon_sq {
            IntrDomainPosition::Snap {
                point: endpoint,
                path_distance,
            }
        } else {
            IntrDomainPosition::Outside
        }
    }
}
