use crate::core::{math::Vector2, traits::Real};

/// A polyline vertex is represented by an `x`, `y`, and `bulge` value.
///
/// `x` and `y` describe the 2D position of the vertex. `bulge` describes the arc sweep angle for
/// the polyline segment that starts with this vertex. `bulge` is defined as
/// `tan(arc_sweep_angle / 4)`. Note a polyline arc segment can never have a sweep angle greater
/// than `PI` (half circle).
///
/// See [angle_from_bulge](crate::core::math::angle_from_bulge) and
/// [bulge_from_angle](crate::core::math::bulge_from_angle) for functions to convert between bulge
/// arc sweep angle.
#[derive(Debug, Copy, Clone, Default, PartialEq)]
pub struct PlineVertex<T = f64> {
    /// X coordinate position for the vertex.
    pub x: T,
    /// Y coordinate position for the vertex.
    pub y: T,
    /// Bulge for the polyline segment that starts with this vertex.
    pub bulge: T,
}

impl<T> PlineVertex<T>
where
    T: Real,
{
    #[inline]
    pub fn new(x: T, y: T, bulge: T) -> Self {
        PlineVertex { x, y, bulge }
    }

    /// Construct a vertex from a [x, y, bulge] slice.
    ///
    ///If the slice does not contain exactly 3 elements then `None` is returned.
    #[inline]
    pub fn from_slice(slice: &[T]) -> Option<Self> {
        if let [x, y, bulge] = *slice {
            Some(PlineVertex::new(x, y, bulge))
        } else {
            None
        }
    }

    /// Construct a vertex using a 2D vector as the position.
    #[inline]
    pub fn from_vector2(vector2: Vector2<T>, bulge: T) -> Self {
        PlineVertex::new(vector2.x, vector2.y, bulge)
    }

    /// Return the position as a 2D vector.
    #[inline]
    pub fn pos(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }

    /// Returns true if `self.bulge.fuzzy_eq_zero()` (represents the start of a line segment).
    #[inline]
    pub fn bulge_is_zero(&self) -> bool {
        self.bulge.fuzzy_eq_zero()
    }

    /// Returns true if `self.bulge > T::zero()` (represents the start of a counter clockwise arc
    /// segment).
    #[inline]
    pub fn bulge_is_pos(&self) -> bool {
        self.bulge > T::zero()
    }

    /// Returns true if `self.bulge < T::zero()` (represents the start of a clockwise arc segment).
    #[inline]
    pub fn bulge_is_neg(&self) -> bool {
        self.bulge < T::zero()
    }

    /// Fuzzy equal comparison with another vertex using `fuzzy_epsilon` given.
    #[inline]
    pub fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: T) -> bool {
        self.x.fuzzy_eq_eps(other.x, fuzzy_epsilon)
            && self.y.fuzzy_eq_eps(other.y, fuzzy_epsilon)
            && self.bulge.fuzzy_eq_eps(other.bulge, fuzzy_epsilon)
    }

    /// Fuzzy equal comparison with another vertex using T::fuzzy_epsilon().
    #[inline]
    pub fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, T::fuzzy_epsilon())
    }
}
