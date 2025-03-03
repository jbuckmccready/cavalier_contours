mod demo_scenes_pane;
mod multi_pline_offset_scene;
mod pline_boolean_scene;
mod pline_offset_scene;

pub use demo_scenes_pane::DemoScenes;

pub trait Scene {
    fn name(&self) -> &str;

    fn ui(&mut self, ui: &mut egui::Ui, init: bool);
}
