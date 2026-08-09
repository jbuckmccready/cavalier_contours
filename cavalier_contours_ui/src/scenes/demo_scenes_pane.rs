use egui::{RichText, Ui};

use super::Scene;
use super::scene_settings::SceneSettings;

pub struct DemoScenes {
    settings: SceneSettings,
    settings_open: bool,
    scenes: Vec<Box<dyn Scene>>,
    selected: Option<usize>,
}

impl Default for DemoScenes {
    fn default() -> Self {
        Self::from_scenes(vec![
            Box::new(super::pline_offset_scene::PlineOffsetScene::default()),
            Box::new(super::pline_boolean_scene::PlineBooleanScene::default()),
            Box::new(super::multi_pline_offset_scene::MultiPlineOffsetScene::default()),
        ])
    }
}

impl DemoScenes {
    pub fn from_scenes(scenes: Vec<Box<dyn Scene>>) -> Self {
        Self {
            settings: SceneSettings::default(),
            settings_open: false,
            scenes,
            selected: None,
        }
    }

    pub fn ui(&mut self, ui: &mut Ui, _frame: &eframe::Frame) {
        let Self {
            settings,
            settings_open,
            scenes,
            selected,
            ..
        } = self;

        // Apply theme to the root UI and egui context.
        let ctx = ui.ctx().clone();
        let visuals = settings.theme.to_egui_visuals(&ctx);
        ctx.set_visuals(visuals.clone());
        ui.visuals_mut().clone_from(&visuals);

        let selected_before = *selected;
        let mut sel = selected.unwrap_or(0);
        egui::Panel::top("menu_bar").show(ui, |ui| {
            ui.horizontal(|ui| {
                use egui::special_emojis::GITHUB;
                ui.hyperlink_to(
                    RichText::new(format!("{GITHUB} Repo")),
                    "https://github.com/jbuckmccready/cavalier_contours",
                );

                ui.toggle_value(settings_open, "🔧 Settings");
                settings.show(&ctx, settings_open);
            });
            ui.separator();
            ui.horizontal(|ui| {
                for (i, scene) in scenes.iter().enumerate() {
                    ui.selectable_value(&mut sel, i, RichText::new(scene.name()).heading());
                }
            })
        });
        *selected = Some(sel);

        let fill = ui.visuals().extreme_bg_color;
        let frame = egui::Frame::NONE.fill(fill);
        egui::CentralPanel::default().frame(frame).show(ui, |ui| {
            scenes[selected.unwrap_or(0)].ui(ui, settings, *selected != selected_before);
        });
    }
}
