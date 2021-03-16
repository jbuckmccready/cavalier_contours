//! Core/common traits for use in cavalier_contours.
mod fuzzy_eq;
mod fuzzy_ord;
mod real;

pub use fuzzy_eq::FuzzyEq;
pub use fuzzy_ord::FuzzyOrd;
pub use real::Real;
