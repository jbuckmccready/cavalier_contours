mod demo_scenes_pane;
mod pline_boolean_scene;
mod pline_offset_scene;

use cavalier_contours::static_aabb2d_index::AABB;
pub use demo_scenes_pane::DemoScenes;

pub trait Scene {
    fn name(&self) -> &str;

    fn ui(&mut self, ui: &mut egui::Ui, init: bool);
}

/// Helper to build a bounding box AABB around a set of AABBs.
fn total_bounds(bounds: impl IntoIterator<Item = AABB>) -> Option<AABB> {
    bounds.into_iter().fold(None, |acc, e| {
        acc.map(|a| AABB {
            min_x: a.min_x.min(e.min_x),
            min_y: a.min_y.min(e.min_y),
            max_x: a.max_x.max(e.max_x),
            max_y: a.max_y.max(e.max_y),
        })
        .or(Some(e))
    })
}
