use crate::{Real, Vector2};

#[derive(Debug, Copy, Clone)]
pub struct PlineVertex<T = f64> {
    pub x: T,
    pub y: T,
    pub bulge: T,
}

impl<T> PlineVertex<T>
where
    T: Real,
{
    pub fn new(x: T, y: T, bulge: T) -> Self {
        PlineVertex { x, y, bulge }
    }

    pub fn from_slice(slice: &[T]) -> Option<Self> {
        if let &[x, y, bulge] = slice {
            Some(PlineVertex::new(x, y, bulge))
        } else {
            None
        }
    }

    pub fn from_vector2(vector2: Vector2<T>, bulge: T) -> Self {
        PlineVertex::new(vector2.x, vector2.y, bulge)
    }

    pub fn pos(&self) -> Vector2<T> {
        Vector2::new(self.x, self.y)
    }

    pub fn bulge_is_zero(&self) -> bool {
        self.bulge.fuzzy_eq_zero()
    }

    pub fn bulge_is_pos(&self) -> bool {
        self.bulge > T::zero()
    }

    pub fn bulge_is_neg(&self) -> bool {
        self.bulge < T::zero()
    }

    /// Fuzzy equal comparison with another vertex using `fuzzy_epsilon` given.
    pub fn fuzzy_eq_eps(&self, other: Self, fuzzy_epsilon: T) -> bool {
        self.x.fuzzy_eq_eps(other.x, fuzzy_epsilon)
            && self.y.fuzzy_eq_eps(other.y, fuzzy_epsilon)
            && self.bulge.fuzzy_eq_eps(other.bulge, fuzzy_epsilon)
    }

    /// Fuzzy equal comparison with another vertex using T::fuzzy_epsilon().
    pub fn fuzzy_eq(&self, other: Self) -> bool {
        self.fuzzy_eq_eps(other, T::fuzzy_epsilon())
    }
}

#[inline(always)]
pub fn pline_vert<T>(x: T, y: T, bulge: T) -> PlineVertex<T>
where
    T: Real,
{
    PlineVertex::new(x, y, bulge)
}
