//! Interactive egui demo application for cavalier_contours.

mod app;
pub use app::MainApp;
/// Shared polyline editing UI used by the demo scenes.
pub mod editor;
/// Plot rendering helpers for polylines, shapes, and offset diagnostics.
pub mod plotting;
mod scenes;
/// Theme definitions used by the demo UI.
pub mod theme;
