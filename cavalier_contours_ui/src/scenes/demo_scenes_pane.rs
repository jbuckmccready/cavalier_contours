use eframe::egui;
use eframe::egui::Context;

use super::Scene;

pub struct DemoScenes {
    //#[cfg_attr(feature = "serde", serde(skip))]
    scenes: Vec<Box<dyn Scene>>,
    selected: Option<usize>,
}

impl Default for DemoScenes {
    fn default() -> Self {
        Self::from_scenes(vec![
            Box::new(super::pline_offset_scene::PlineOffsetScene::default()),
            Box::new(super::pline_boolean_scene::PlineBooleanScene::default()),
        ])
    }
}

impl DemoScenes {
    pub fn from_scenes(scenes: Vec<Box<dyn Scene>>) -> Self {
        Self {
            scenes,
            selected: None,
        }
    }

    pub fn ui(&mut self, ctx: &Context, _epi_frame: &eframe::Frame) {
        let Self { scenes, selected } = self;

        let selected_before = *selected;
        let mut sel = selected.unwrap_or(0);
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                for (i, scene) in scenes.iter().enumerate() {
                    ui.selectable_value(&mut sel, i, scene.name());
                }
            })
        });
        *selected = Some(sel);

        let fill = ctx.style().visuals.extreme_bg_color;
        let frame = egui::Frame::NONE.fill(fill);
        egui::CentralPanel::default().frame(frame).show(ctx, |ui| {
            scenes[selected.unwrap_or(0)].ui(ui, *selected != selected_before);
        });
    }
}
