use crate::core::traits::Real;
use std::ops;

#[derive(Debug, Copy, Clone, PartialEq)]
pub struct Vector2<T = f64> {
    pub x: T,
    pub y: T,
}

impl<T> Vector2<T>
where
    T: Real,
{
    /// Create a new vector with x and y components.
    pub fn new(x: T, y: T) -> Self {
        Vector2 { x, y }
    }

    /// Create a zero vector (x = 0, y = 0).
    pub fn zero() -> Self {
        Vector2::new(T::zero(), T::zero())
    }

    /// Uniformly scale the vector by `scale_factor`.
    pub fn scale(&self, scale_factor: T) -> Self {
        vec2(scale_factor * self.x, scale_factor * self.y)
    }

    /// Dot product.
    pub fn dot(&self, other: Self) -> T {
        self.x * other.x + self.y * other.y
    }

    /// Compute the perpendicular dot product (`self.x * other.y - self.y * other.x`).
    pub fn perp_dot(&self, other: Self) -> T {
        self.x * other.y - self.y * other.x
    }

    /// Squared length of the vector.
    pub fn length_squared(&self) -> T {
        self.dot(*self)
    }

    /// Length of the vector.
    pub fn length(&self) -> T {
        self.dot(*self).sqrt()
    }

    /// Normalize the vector (length = 1).
    pub fn normalize(&self) -> Self {
        self.scale(T::one() / self.length())
    }

    /// Fuzzy equal comparison with another vector using `fuzzy_epsilon` given.
    pub fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: T) -> bool {
        self.x.fuzzy_eq_eps(other.x, fuzzy_epsilon) && self.y.fuzzy_eq_eps(other.y, fuzzy_epsilon)
    }

    /// Fuzzy equal comparison with another vector using T::fuzzy_epsilon().
    pub fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, T::fuzzy_epsilon())
    }

    /// Create perpendicular vector.
    pub fn perp(&self) -> Self {
        vec2(-self.y, self.x)
    }

    /// Create perpendicular unit vector (length = 1).
    pub fn unit_perp(&self) -> Self {
        self.perp().normalize()
    }

    /// Rotate this point around an `origin` point by some `angle` in radians.
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
            fn $op_func(self, rhs: Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op rhs.x, self.y $op rhs.y)
            }
        }

        impl<T: Real> ops::$op_trait<&Vector2<T>> for Vector2<T> {
            type Output = Vector2<T>;
            fn $op_func(self, rhs: &Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op rhs.x, self.y $op rhs.y)
            }
        }


        impl<'a, 'b, T: Real> ops::$op_trait<&'b Vector2<T>> for &'a Vector2<T> {
            type Output = Vector2<T>;
            fn $op_func(self, _rhs: &'b Vector2<T>) -> Self::Output {
                Vector2::new(self.x $op _rhs.x, self.y $op _rhs.y)
            }
        }

        impl<T: Real> ops::$op_trait<Vector2<T>> for &Vector2<T> {
            type Output = Vector2<T>;
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
            fn $op_func(self) -> Self::Output {
                Vector2::new($op self.x, $op self.y)
            }
        }

        impl<T: Real> ops::$op_trait for &Vector2<T> {
            type Output = Vector2<T>;
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
            assert!(($v1 $op $v2).fuzzy_eq($expected));
            assert!((&$v1 $op $v2).fuzzy_eq($expected));
            assert!(($v1 $op &$v2).fuzzy_eq($expected));
            assert!((&$v1 $op &$v2).fuzzy_eq($expected));
        };
    }

    #[test]
    fn ops() {
        let v1 = vec2(4.0, 5.0);
        let v2 = vec2(1.0, 2.0);
        test_binary_op!(v1, v2, +, vec2(5.0, 7.0));
        test_binary_op!(v1, v2, -, vec2(3.0, 3.0));
    }
}
