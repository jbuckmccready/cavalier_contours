use crate::core::traits::Real;
use std::{fmt::Display, ops};

/// A 2D vector with x and y components.
///
/// This is the fundamental 2D vector type used throughout the library for representing
/// points, directions, and performing vector operations. The type parameter `T` defaults
/// to `f64` but can be any type that implements the [`Real`] trait.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::{Vector2, vec2};
/// let v1 = Vector2::new(3.0, 4.0);
/// let v2 = vec2(1.0, 2.0); // shorthand constructor
///
/// // Vector operations
/// let sum = v1 + v2;
/// let dot_product = v1.dot(v2);
/// let length = v1.length();
/// let normalized = v1.normalize();
/// ```
#[must_use]
#[derive(Debug, Copy, Clone, Default, PartialEq, Eq)]
pub struct Vector2<T = f64> {
    /// The x-coordinate component.
    pub x: T,
    /// The y-coordinate component.
    pub y: T,
}

impl<T> Display for Vector2<T>
where
    T: Display,
{
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "[{}, {}]", self.x, self.y)
    }
}

impl<T> Vector2<T>
where
    T: Real,
{
    /// Create a new vector with x and y components.
    #[inline]
    pub fn new(x: T, y: T) -> Self {
        Vector2 { x, y }
    }

    /// Create a zero vector (x = 0, y = 0).
    #[inline]
    pub fn zero() -> Self {
        Vector2::new(T::zero(), T::zero())
    }

    /// Uniformly scale the vector by `scale_factor`.
    #[inline]
    pub fn scale(&self, scale_factor: T) -> Self {
        vec2(scale_factor * self.x, scale_factor * self.y)
    }

    /// Dot product.
    #[inline]
    #[must_use]
    pub fn dot(&self, other: Self) -> T {
        self.x * other.x + self.y * other.y
    }

    /// Compute the perpendicular dot product (`self.x * other.y - self.y * other.x`).
    #[inline]
    #[must_use]
    pub fn perp_dot(&self, other: Self) -> T {
        self.x * other.y - self.y * other.x
    }

    /// Squared length of the vector.
    #[inline]
    #[must_use]
    pub fn length_squared(&self) -> T {
        self.dot(*self)
    }

    /// Length of the vector.
    #[inline]
    #[must_use]
    pub fn length(&self) -> T {
        self.dot(*self).sqrt()
    }

    /// Normalize the vector (length = 1).
    #[inline]
    pub fn normalize(&self) -> Self {
        self.scale(T::one() / self.length())
    }

    /// Compares this vector with another using Euclidean distance.
    ///
    /// Returns `true` when the squared Euclidean distance is strictly less than
    /// `fuzzy_epsilon * fuzzy_epsilon`. `fuzzy_epsilon` must be greater than zero.
    ///
    /// # Panics
    ///
    /// Panics in debug builds if `fuzzy_epsilon` is zero or negative.
    #[inline]
    #[must_use]
    pub fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: T) -> bool {
        debug_assert!(
            fuzzy_epsilon > T::zero(),
            "fuzzy_epsilon must be greater than zero"
        );
        (*self - other).length_squared() < fuzzy_epsilon * fuzzy_epsilon
    }

    /// Compares this vector with another using Euclidean distance and `T::fuzzy_epsilon()`.
    #[inline]
    #[must_use]
    pub fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, T::fuzzy_epsilon())
    }

    /// Create perpendicular vector.
    #[inline]
    pub fn perp(&self) -> Self {
        vec2(-self.y, self.x)
    }

    /// Create perpendicular unit vector (length = 1).
    #[inline]
    pub fn unit_perp(&self) -> Self {
        self.perp().normalize()
    }

    /// Rotate this point around an `origin` point by some `angle` in radians.
    #[inline]
    pub fn rotate_about(&self, origin: Self, angle: T) -> Self {
        // translate to origin
        let translated = self - origin;

        // rotate
        let s = angle.sin();
        let c = angle.cos();
        let rotated = vec2(
            translated.x * c - translated.y * s,
            translated.x * s + translated.y * c,
        );

        // translate back
        rotated + origin
    }
}

#[cfg(feature = "serde")]
mod serde_impl {
    use super::Vector2;
    use serde::{Deserialize, Serialize, ser::SerializeTuple};

    impl<T> Serialize for Vector2<T>
    where
        T: Serialize,
    {
        fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
        where
            S: serde::Serializer,
        {
            let mut tuple = serializer.serialize_tuple(3)?;
            tuple.serialize_element(&self.x)?;
            tuple.serialize_element(&self.y)?;
            tuple.end()
        }
    }

    impl<'de, T> Deserialize<'de> for Vector2<T>
    where
        T: Deserialize<'de>,
    {
        fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
        where
            D: serde::Deserializer<'de>,
        {
            let [x, y]: [T; 2] = Deserialize::deserialize(deserializer)?;
            Ok(Vector2 { x, y })
        }
    }
}
#[cfg(feature = "serde")]
// Re-export trait implementations for serde support
// Note: clippy may warn about unused imports, but these are needed for trait impls
#[allow(unused_imports)]
pub use serde_impl::*;

/// Shorthand constructor for creating a [`Vector2`].
///
/// This is a convenience function equivalent to `Vector2::new(x, y)`.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::math::vec2;
/// let v = vec2(3.0, 4.0);
/// assert_eq!(v.x, 3.0);
/// assert_eq!(v.y, 4.0);
/// ```
#[inline(always)]
pub fn vec2<T>(x: T, y: T) -> Vector2<T>
where
    T: Real,
{
    Vector2::new(x, y)
}

macro_rules! ImplBinaryOp {
    ($op_trait:ident, $op_func:ident, $op:tt) => {
        impl<T: Real> ops::$op_trait<Vector2<T>> for Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self, rhs: Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op rhs.x, self.y $op rhs.y)
            }
        }

        impl<T: Real> ops::$op_trait<&Vector2<T>> for Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self, rhs: &Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op rhs.x, self.y $op rhs.y)
            }
        }


        impl<'a, 'b, T: Real> ops::$op_trait<&'b Vector2<T>> for &'a Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self, _rhs: &'b Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op _rhs.x, self.y $op _rhs.y)
            }
        }

        impl<T: Real> ops::$op_trait<Vector2<T>> for &Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self, rhs: Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op rhs.x, self.y $op rhs.y)
            }
        }
    };
}

ImplBinaryOp!(Add, add, +);
ImplBinaryOp!(Sub, sub, -);

macro_rules! ImplUnaryOp {
    ($op_trait:ident, $op_func:ident, $op:tt) => {
        impl<T: Real> ops::$op_trait for Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self) -> Self::Output {
                Vector2::new($op self.x, $op self.y)
            }
        }

        impl<T: Real> ops::$op_trait for &Vector2<T> {
            type Output = Vector2<T>;
            #[inline]
            fn $op_func(self) -> Self::Output {
                Vector2::new($op self.x, $op self.y)
            }
        }

    };
}

ImplUnaryOp!(Neg, neg, -);

#[cfg(test)]
mod tests {
    use super::*;

    macro_rules! test_binary_op {
        ($v1:ident, $v2:ident, $op:tt, $expected:expr) => {
            assert_eq!($v1 $op $v2, $expected);
            assert_eq!(&$v1 $op $v2, $expected);
            assert_eq!($v1 $op &$v2, $expected);
            assert_eq!(&$v1 $op &$v2, $expected);
        };
    }

    macro_rules! test_unary_op {
        ($v:ident, $op:tt, $expected:expr) => {
            assert_eq!($op $v, $expected);
            assert_eq!($op &$v, $expected);
        };
    }

    #[test]
    fn operators_support_owned_and_borrowed_operands() {
        let v1 = vec2(4.0, 5.0);
        let v2 = vec2(1.0, 2.0);
        test_binary_op!(v1, v2, +, vec2(5.0, 7.0));
        test_binary_op!(v1, v2, -, vec2(3.0, 3.0));
        test_unary_op!(v1, -, vec2(-4.0, -5.0));
    }

    #[test]
    fn fuzzy_eq_eps_has_strict_axis_boundary() {
        let origin = Vector2::zero();
        let epsilon = 1.0;

        assert!(origin.fuzzy_eq_eps(Vector2::new(1023.0 / 1024.0, 0.0), epsilon));
        assert!(!origin.fuzzy_eq_eps(Vector2::new(1.0, 0.0), epsilon));
        assert!(!origin.fuzzy_eq_eps(Vector2::new(1025.0 / 1024.0, 0.0), epsilon));
    }

    #[test]
    fn fuzzy_eq_eps_uses_euclidean_distance() {
        assert!(!Vector2::zero().fuzzy_eq_eps(Vector2::new(0.75, 0.75), 1.0));
    }

    #[test]
    fn fuzzy_eq_eps_has_strict_three_four_five_boundary() {
        let origin = Vector2::zero();
        let epsilon = 5.0;

        assert!(origin.fuzzy_eq_eps(
            Vector2::new(3.0 * 1023.0 / 1024.0, 4.0 * 1023.0 / 1024.0),
            epsilon
        ));
        assert!(!origin.fuzzy_eq_eps(Vector2::new(3.0, 4.0), epsilon));
        assert!(!origin.fuzzy_eq_eps(
            Vector2::new(3.0 * 1025.0 / 1024.0, 4.0 * 1025.0 / 1024.0),
            epsilon
        ));
    }

    #[test]
    fn fuzzy_eq_eps_is_symmetric() {
        let point = Vector2::new(2.0, -3.0);
        let inside = Vector2::new(2.5, -2.5);
        let outside = Vector2::new(2.75, -2.25);

        assert!(point.fuzzy_eq_eps(inside, 1.0));
        assert!(inside.fuzzy_eq_eps(point, 1.0));
        assert!(!point.fuzzy_eq_eps(outside, 1.0));
        assert!(!outside.fuzzy_eq_eps(point, 1.0));
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "fuzzy_epsilon must be greater than zero")]
    fn fuzzy_eq_eps_debug_asserts_zero_epsilon() {
        let _ = Vector2::zero().fuzzy_eq_eps(Vector2::zero(), 0.0);
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "fuzzy_epsilon must be greater than zero")]
    fn fuzzy_eq_eps_debug_asserts_negative_epsilon() {
        let _ = Vector2::zero().fuzzy_eq_eps(Vector2::zero(), -1.0);
    }

    #[test]
    fn fuzzy_eq_eps_handles_representable_offsets_after_large_translation() {
        let base = 1_099_511_627_776.0;
        let point = Vector2::new(base, -base);

        assert!(point.fuzzy_eq_eps(Vector2::new(base + 0.5, -base + 0.5), 1.0));
        assert!(!point.fuzzy_eq_eps(Vector2::new(base + 0.75, -base + 0.75), 1.0));
    }
}
