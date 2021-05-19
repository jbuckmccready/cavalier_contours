//! Core module has common/shared math, traits, and utility modules.

use self::traits::ControlFlow;
pub mod math;
pub mod traits;

/// Basic control flow enum that can be used when visiting query results.
#[derive(Debug)]
pub enum Control<B = ()> {
    /// Indicates to the query function to continue visiting results.
    Continue,
    /// Indicates to the query function to stop visiting results and return a value.
    Break(B),
}

impl<B> Default for Control<B> {
    #[inline]
    fn default() -> Self {
        Control::Continue
    }
}

impl<B> ControlFlow for Control<B> {
    #[inline]
    fn continuing() -> Self {
        Control::Continue
    }

    #[inline]
    fn should_break(&self) -> bool {
        matches!(*self, Control::Break(_))
    }
}

/// Internal macro used for try return on control flow.
macro_rules! try_cf {
    ($e:expr) => {
        match $e {
            x => {
                if x.should_break() {
                    return x;
                }
            }
        }
    };
}
