use super::{PlineVertex, seg_arc_radius_and_center};
use crate::core::{
    math::{
        Circle, IntersectionPoint, Line, Vector2, angle, angle_from_bulge, dist_squared,
        parametric_from_point_unchecked, point_from_parametric,
    },
    traits::Real,
};

#[derive(Debug, Copy, Clone)]
pub(crate) struct SegmentLocation<T>
where
    T: Real,
{
    pub(crate) t: T,
    pub(crate) point: Vector2<T>,
    pub(crate) adjustment_error_squared: T,
}

/// A finite directed line segment.
#[derive(Debug, Copy, Clone)]
pub struct LineSegment<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
}

impl<T> LineSegment<T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn as_line(&self) -> Option<Line<T>> {
        let direction = self.end - self.start;
        (direction.x != T::zero() || direction.y != T::zero()).then_some(Line {
            point: self.start,
            direction,
        })
    }

    #[inline]
    #[must_use]
    pub fn parameter_at(&self, point: Vector2<T>) -> T {
        parametric_from_point_unchecked(self.start, self.end, point)
    }

    #[inline]
    pub fn point_at(&self, t: T) -> Vector2<T> {
        if t == T::zero() {
            self.start
        } else if t == T::one() {
            self.end
        } else {
            point_from_parametric(self.start, self.end, t)
        }
    }

    #[inline(always)]
    pub(crate) fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>> {
        let direction = self.end - self.start;
        let length_squared = direction.length_squared();
        if length_squared == T::zero() {
            let adjustment_error_squared = dist_squared(candidate.point, self.start);
            return (adjustment_error_squared < epsilon_squared).then_some(SegmentLocation {
                t: T::zero(),
                point: self.start,
                adjustment_error_squared,
            });
        }

        let t = self.parameter_at(candidate.point);
        if t < T::zero() {
            let adjustment_error_squared = t * t * length_squared;
            (adjustment_error_squared < epsilon_squared).then_some(SegmentLocation {
                t: T::zero(),
                point: self.start,
                adjustment_error_squared,
            })
        } else if t > T::one() {
            let overshoot = t - T::one();
            let adjustment_error_squared = overshoot * overshoot * length_squared;
            (adjustment_error_squared < epsilon_squared).then_some(SegmentLocation {
                t: T::one(),
                point: self.end,
                adjustment_error_squared,
            })
        } else {
            Some(SegmentLocation {
                t,
                point: candidate.point,
                adjustment_error_squared: T::zero(),
            })
        }
    }
}

/// A finite directed circular arc.
#[derive(Debug, Copy, Clone)]
pub struct ArcSegment<T>
where
    T: Real,
{
    pub start: Vector2<T>,
    pub end: Vector2<T>,
    pub center: Vector2<T>,
    pub radius: T,
    /// Positive sweeps run counterclockwise and negative sweeps run clockwise.
    pub sweep: T,
}

impl<T> ArcSegment<T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn circle(&self) -> Circle<T> {
        Circle {
            center: self.center,
            radius: self.radius,
        }
    }

    #[inline]
    #[must_use]
    pub fn parameter_at(&self, point: Vector2<T>) -> T {
        if point == self.start {
            return T::zero();
        }
        if point == self.end {
            return T::one();
        }
        let start_vector = self.start - self.center;
        let point_vector = point - self.center;
        T::atan2(
            start_vector.perp_dot(point_vector),
            start_vector.dot(point_vector),
        ) / self.sweep
    }

    #[inline]
    pub fn point_at(&self, t: T) -> Vector2<T> {
        if t == T::zero() {
            return self.start;
        }
        if t == T::one() {
            return self.end;
        }
        let (sin, cos) = (angle(self.center, self.start) + self.sweep * t).sin_cos();
        self.center + Vector2::new(self.radius * cos, self.radius * sin)
    }

    #[inline]
    pub(super) fn forward_angle(&self, point: Vector2<T>) -> T {
        let start_vector = self.start - self.center;
        let point_vector = point - self.center;
        let cross = start_vector.perp_dot(point_vector);
        let oriented_cross = if self.sweep < T::zero() {
            -cross
        } else {
            cross
        };
        let mut result = T::atan2(oriented_cross, start_vector.dot(point_vector));
        if result < T::zero() {
            result = result + T::tau();
        }
        result
    }

    #[inline(always)]
    pub(crate) fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>> {
        // Exact whole-circle points already satisfy the radial domain. Adjusted tangencies and
        // coincident-circle boundary projections still need a strict radial check.
        if candidate.error_squared != T::zero() {
            let radial_difference = (candidate.point - self.center).length() - self.radius;
            if radial_difference * radial_difference >= epsilon_squared {
                return None;
            }
        }

        let forward_angle = self.forward_angle(candidate.point);
        let sweep = self.sweep.abs();
        if forward_angle <= sweep {
            return Some(SegmentLocation {
                t: forward_angle / sweep,
                point: candidate.point,
                adjustment_error_squared: T::zero(),
            });
        }

        let before_start = T::tau() - forward_angle;
        let after_end = forward_angle - sweep;
        let (overshoot, point, t) = if before_start < after_end {
            (before_start * self.radius, self.start, T::zero())
        } else {
            (after_end * self.radius, self.end, T::one())
        };
        let adjustment_error_squared = dist_squared(candidate.point, point);
        if overshoot * overshoot < epsilon_squared && adjustment_error_squared < epsilon_squared {
            Some(SegmentLocation {
                t,
                point,
                adjustment_error_squared,
            })
        } else {
            None
        }
    }
}

/// A finite polyline segment with its line or arc geometry already resolved.
#[derive(Debug, Copy, Clone)]
pub enum Segment<T>
where
    T: Real,
{
    Line(LineSegment<T>),
    Arc(ArcSegment<T>),
}

impl<T> Segment<T>
where
    T: Real,
{
    #[inline]
    #[must_use]
    pub fn from_vertices(start: PlineVertex<T>, end: PlineVertex<T>) -> Self {
        if start.bulge_is_zero() || start.pos() == end.pos() {
            return Self::Line(LineSegment {
                start: start.pos(),
                end: end.pos(),
            });
        }
        let (radius, center) = seg_arc_radius_and_center(start, end);
        Self::Arc(ArcSegment {
            start: start.pos(),
            end: end.pos(),
            center,
            radius,
            sweep: angle_from_bulge(start.bulge),
        })
    }

    #[inline(always)]
    pub(crate) fn locate(
        &self,
        candidate: IntersectionPoint<T>,
        epsilon_squared: T,
    ) -> Option<SegmentLocation<T>> {
        match self {
            Self::Line(line) => line.locate(candidate, epsilon_squared),
            Self::Arc(arc) => arc.locate(candidate, epsilon_squared),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn arc(start_angle: f64, sweep: f64) -> ArcSegment<f64> {
        let point_at = |angle: f64| Vector2::new(angle.cos(), angle.sin());
        ArcSegment {
            start: point_at(start_angle),
            end: point_at(start_angle + sweep),
            center: Vector2::zero(),
            radius: 1.0,
            sweep,
        }
    }

    #[test]
    fn line_parameters_follow_direction() {
        let line = LineSegment::<f64> {
            start: Vector2::new(2.0, 0.0),
            end: Vector2::zero(),
        };
        assert!((line.parameter_at(Vector2::new(1.5, 0.0)) - 0.25).abs() < f64::EPSILON);
        assert_eq!(line.point_at(0.25), Vector2::new(1.5, 0.0));
    }

    #[test]
    fn zero_length_line_has_no_whole_line() {
        let line = LineSegment::<f64> {
            start: Vector2::zero(),
            end: Vector2::zero(),
        };
        assert!(line.as_line().is_none());
    }

    #[test]
    fn arc_parameters_follow_positive_and_negative_sweeps() {
        for sweep in [FRAC_PI_2, -FRAC_PI_2] {
            let arc = arc(0.25, sweep);
            for t in [0.0, 0.25, 0.5, 0.75, 1.0] {
                let point = arc.point_at(t);
                assert!((arc.parameter_at(point) - t).abs() < 1e-14);
            }
        }
    }

    #[test]
    fn arc_parameters_handle_angle_wrap() {
        let arc = arc(PI - 0.25, 0.5);
        assert!((arc.parameter_at(arc.point_at(0.75)) - 0.75).abs() < 1e-14);
    }

    #[test]
    fn arc_point_at_preserves_exact_endpoints() {
        let arc = ArcSegment {
            start: Vector2::new(1.0, 1e-16),
            end: Vector2::new(0.0, 1.0),
            center: Vector2::zero(),
            radius: 1.0,
            sweep: FRAC_PI_2,
        };
        assert_eq!(arc.point_at(0.0), arc.start);
        assert_eq!(arc.point_at(1.0), arc.end);
        assert!(arc.parameter_at(arc.start).abs() < f64::EPSILON);
        assert!((arc.parameter_at(arc.end) - 1.0).abs() < f64::EPSILON);
    }
}
