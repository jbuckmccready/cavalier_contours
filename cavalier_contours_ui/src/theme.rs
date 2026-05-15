use egui::Color32;
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize, Default)]
pub enum Theme {
    Light,
    Dark,
    #[default]
    System,
}

impl Theme {
    pub fn label(&self) -> &'static str {
        match self {
            Theme::Light => "☀ Light",
            Theme::Dark => "🌙 Dark",
            Theme::System => "🖥 System",
        }
    }

    /// Helper method to determine if system theme should use dark mode
    /// Defaults to dark when system theme detection fails
    fn system_is_dark(ctx: &egui::Context) -> bool {
        match ctx.system_theme() {
            Some(egui::Theme::Light) => false,
            Some(egui::Theme::Dark) | None => true,
        }
    }

    pub fn to_egui_visuals(&self, ctx: &egui::Context) -> egui::Visuals {
        match self {
            Theme::Light => egui::Visuals::light(),
            Theme::Dark => egui::Visuals::dark(),
            Theme::System => {
                if Self::system_is_dark(ctx) {
                    egui::Visuals::dark()
                } else {
                    egui::Visuals::light()
                }
            }
        }
    }

    pub fn colors(&self, ctx: &egui::Context) -> ThemeColors {
        match self {
            Theme::Light => ThemeColors::light(),
            Theme::Dark => ThemeColors::dark(),
            Theme::System => {
                if Self::system_is_dark(ctx) {
                    ThemeColors::dark()
                } else {
                    ThemeColors::light()
                }
            }
        }
    }
}

/// High-contrast color palette for both light and dark themes.
///
/// This struct provides semantic color definitions that automatically adapt
/// to the current theme, ensuring optimal contrast and readability.
#[derive(Debug, Clone)]
pub struct ThemeColors {
    /// Primary stroke color for main geometric elements
    pub primary_stroke: Color32,
    /// Secondary stroke color for secondary geometric elements
    pub secondary_stroke: Color32,
    /// Accent stroke color for highlighted elements
    pub accent_stroke: Color32,
    /// Color for vertex points and markers
    pub vertex_color: Color32,
    /// Fill color for filled shapes
    pub fill_color: Color32,

    /// Multi-polyline colors for distinguishing multiple shapes
    pub multi_colors: [Color32; 6],

    /// Error message text color
    pub error_color: Color32,
    /// Warning message text color
    pub warning_color: Color32,
    /// Success message text color
    pub success_color: Color32,
    /// Information message text color
    pub info_color: Color32,

    /// Color for raw offset operation segments
    pub raw_offset_color: Color32,
    /// Color for collapsed segments in offset operations
    pub collapsed_color: Color32,
    /// Color for grid lines in the plot area
    pub grid_color: Color32,
}

impl ThemeColors {
    fn light() -> Self {
        Self {
            // Primary colors - high contrast on light background
            primary_stroke: Color32::from_rgb(0, 100, 200), // Deep blue
            secondary_stroke: Color32::from_rgb(200, 50, 50), // Deep red
            accent_stroke: Color32::from_rgb(200, 140, 0),  // Dark gold
            vertex_color: Color32::from_rgb(0, 150, 0),     // Dark green
            fill_color: Color32::from_rgba_unmultiplied(200, 200, 255, 60), // Light blue fill

            // Multi-polyline palette - carefully chosen for contrast
            multi_colors: [
                Color32::from_rgb(0, 100, 200),  // Deep blue
                Color32::from_rgb(200, 50, 50),  // Deep red
                Color32::from_rgb(0, 150, 0),    // Dark green
                Color32::from_rgb(150, 0, 150),  // Purple
                Color32::from_rgb(200, 100, 0),  // Orange
                Color32::from_rgb(100, 50, 150), // Violet
            ],

            // UI colors
            error_color: Color32::from_rgb(180, 0, 0), // Dark red
            warning_color: Color32::from_rgb(180, 120, 0), // Dark amber
            success_color: Color32::from_rgb(0, 120, 0), // Dark green
            info_color: Color32::from_rgb(0, 80, 160), // Dark blue

            // Specialized colors
            raw_offset_color: Color32::from_rgb(120, 0, 120), // Dark purple
            collapsed_color: Color32::from_rgb(180, 0, 0),    // Dark red
            grid_color: Color32::from_rgb(180, 180, 180),     // Light gray
        }
    }

    fn dark() -> Self {
        Self {
            // Primary colors - high contrast on dark background
            primary_stroke: Color32::from_rgb(100, 180, 255), // Bright blue
            secondary_stroke: Color32::from_rgb(255, 120, 120), // Bright red
            accent_stroke: Color32::from_rgb(255, 200, 80),   // Bright gold
            vertex_color: Color32::from_rgb(120, 255, 120),   // Bright green
            fill_color: Color32::from_rgba_unmultiplied(100, 150, 255, 60), // Blue fill

            // Multi-polyline palette - bright colors for dark background
            multi_colors: [
                Color32::from_rgb(100, 180, 255), // Bright blue
                Color32::from_rgb(255, 120, 120), // Bright red
                Color32::from_rgb(120, 255, 120), // Bright green
                Color32::from_rgb(255, 120, 255), // Bright magenta
                Color32::from_rgb(255, 180, 80),  // Bright orange
                Color32::from_rgb(180, 120, 255), // Bright violet
            ],

            // UI colors
            error_color: Color32::from_rgb(255, 100, 100), // Bright red
            warning_color: Color32::from_rgb(255, 200, 80), // Bright amber
            success_color: Color32::from_rgb(120, 255, 120), // Bright green
            info_color: Color32::from_rgb(120, 180, 255),  // Bright blue

            // Specialized colors
            raw_offset_color: Color32::from_rgb(200, 120, 255), // Bright purple
            collapsed_color: Color32::from_rgb(255, 120, 120),  // Bright red
            grid_color: Color32::from_rgb(80, 80, 80),          // Dark gray
        }
    }

    /// Get a color from the multi-color palette by index
    pub fn get_multi_color(&self, index: usize) -> Color32 {
        self.multi_colors[index % self.multi_colors.len()]
    }
}
