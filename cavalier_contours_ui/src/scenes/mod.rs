mod demo_scenes_pane;
mod multi_pline_offset_scene;
mod pline_boolean_scene;
mod pline_offset_scene;
mod scene_settings;
mod shape_boolean_scene;

pub use demo_scenes_pane::DemoScenes;
use scene_settings::SceneSettings;

pub trait Scene {
    fn name(&self) -> &str;

    fn ui(&mut self, ui: &mut egui::Ui, settings: &SceneSettings, init: bool);
}
