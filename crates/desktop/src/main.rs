
use bevy::prelude::*;

struct DesktopPlugin;

impl Plugin for DesktopPlugin {
    fn build(&self, _app: &mut App) {}
}

fn main() {
    app::run_app(DesktopPlugin);
}
