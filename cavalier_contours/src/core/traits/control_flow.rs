use static_aabb2d_index as aabb_index;
/// Trait for control flow inside visiting methods.
pub trait ControlFlow {
    /// Constructs state indicating to continue.
    fn continuing() -> Self;
    /// Should return true if control flow should break.
    fn should_break(&self) -> bool;
}

impl<C> ControlFlow for C
where
    C: aabb_index::ControlFlow,
{
    #[inline]
    fn continuing() -> Self {
        C::continuing()
    }

    #[inline]
    fn should_break(&self) -> bool {
        self.should_break()
    }
}
