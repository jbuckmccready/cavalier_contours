use static_aabb2d_index as aabb_index;
/// Trait for control flow inside visiting methods.
///
/// This trait provides a way to control iteration and visiting patterns in
/// spatial queries and other algorithms. It allows early termination of
/// operations when a desired condition is met.
///
/// # Examples
///
/// ```
/// # use cavalier_contours::core::*;
/// # use cavalier_contours::polyline::*;
/// let mut pline = Polyline::new();
/// pline.add(0.0, 0.0, 0.0);
/// pline.add(2.0, 2.0, 0.0);
/// pline.add(0.0, 2.0, 0.0);
/// pline.add(1.0, 1.0, 0.0);
/// pline.add(-1.0, 1.0, 0.0);
///
/// let mut visited_intersects = 0;
/// pline.visit_self_intersects(&mut |_intersect| {
///     visited_intersects += 1;
///     // Return Control::Break to stop iteration early
///     Control::Break(())
/// });
///
/// assert_eq!(visited_intersects, 1);
/// ```
pub trait ControlFlow {
    /// Constructs state indicating to continue iteration/visiting.
    fn continuing() -> Self;

    /// Returns `true` if control flow should break/stop iteration.
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
