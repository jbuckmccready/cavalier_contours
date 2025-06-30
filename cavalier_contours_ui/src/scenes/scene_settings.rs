use crate::theme::Theme;

pub struct SceneSettings {
    pub show_grid: bool,
    pub show_axes: bool,
    pub show_cursor_pos: bool,
    pub theme: Theme,
}

impl Default for SceneSettings {
    fn default() -> Self {
        Self {
            show_grid: true,
            show_axes: true,
            show_cursor_pos: true,
            theme: Theme::default(),
        }
    }
}

impl SceneSettings {
    pub fn show(&mut self, ctx: &egui::Context, open: &mut bool) {
        egui::Window::new("Settings")
            .open(open)
            .min_width(375.0)
            .default_size([390.0, 500.0])
            .scroll(false)
            .resizable([false, false])
            .show(ctx, |ui| self.ui(ui));
    }

    pub fn apply_to_plot<'a>(&self, plot: egui_plot::Plot<'a>) -> egui_plot::Plot<'a> {
        plot.show_grid([self.show_grid; 2])
            .show_axes(self.show_axes)
            .show_x(self.show_cursor_pos)
            .show_y(self.show_cursor_pos)
    }

    pub fn colors(&self, ctx: &egui::Context) -> crate::theme::ThemeColors {
        self.theme.colors(ctx)
    }

    fn ui(&mut self, ui: &mut egui::Ui) {
        ui.checkbox(&mut self.show_grid, "Show Grid")
            .on_hover_text("Show grid lines in the canvas");
        ui.checkbox(&mut self.show_axes, "Show Axes")
            .on_hover_text("Show x and y axes values along edge of the canvas");
        ui.checkbox(&mut self.show_cursor_pos, "Show Cursor Position")
            .on_hover_text("Show the current cursor crosshair with (x, y) position");

        ui.separator();

        ui.horizontal(|ui| {
            ui.label("Theme:");
            egui::ComboBox::from_id_salt("theme_combo")
                .selected_text(self.theme.label())
                .show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.theme, Theme::Light, Theme::Light.label());
                    ui.selectable_value(&mut self.theme, Theme::Dark, Theme::Dark.label());
                    ui.selectable_value(&mut self.theme, Theme::System, Theme::System.label());
                });
        });

        if ui
            .button("Reset Default")
            .on_hover_text("Reset to default settings")
            .clicked()
        {
            *self = Self::default();
        }
    }
}
