use eframe::egui;
use eframe::egui::Context;
use egui::RichText;

use super::Scene;
use super::scene_settings::SceneSettings;

pub struct DemoScenes {
    settings: SceneSettings,
    settings_open: bool,
    //#[cfg_attr(feature = "serde", serde(skip))]
    scenes: Vec<Box<dyn Scene>>,
    selected: Option<usize>,
}

impl Default for DemoScenes {
    fn default() -> Self {
        Self::from_scenes(vec![
            Box::new(super::pline_offset_scene::PlineOffsetScene::default()),
            Box::new(super::pline_boolean_scene::PlineBooleanScene::default()),
            Box::new(super::multi_pline_offset_scene::MultiPlineOffsetScene::default()),
            Box::new(super::multi_pline_boolean_scene::MultiPlineBooleanScene::default()),
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

    pub fn ui(&mut self, ctx: &Context, _epi_frame: &eframe::Frame) {
        let Self {
            settings,
            settings_open,
            scenes,
            selected,
        } = self;

        let selected_before = *selected;
        let mut sel = selected.unwrap_or(0);
        egui::TopBottomPanel::top("menu_bar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                use egui::special_emojis::GITHUB;
                ui.hyperlink_to(
                    RichText::new(format!("{GITHUB} Repo")),
                    "https://github.com/jbuckmccready/cavalier_contours",
                );

                ui.toggle_value(settings_open, "🔧 Settings");
                settings.show(ctx, settings_open);
            });
            ui.separator();
            ui.horizontal(|ui| {
                for (i, scene) in scenes.iter().enumerate() {
                    ui.selectable_value(&mut sel, i, RichText::new(scene.name()).heading());
                }
            })
        });
        *selected = Some(sel);

        let fill = ctx.style().visuals.extreme_bg_color;
        let frame = egui::Frame::NONE.fill(fill);
        egui::CentralPanel::default().frame(frame).show(ctx, |ui| {
            scenes[selected.unwrap_or(0)].ui(ui, settings, *selected != selected_before);
        });
    }
}
