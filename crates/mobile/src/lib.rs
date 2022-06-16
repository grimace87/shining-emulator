
use bevy::prelude::*;

struct MobilePlugin;

impl Plugin for MobilePlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(Msaa::Off);
    }
}

#[bevy_main]
fn main() {
    app::run_app(MobilePlugin);
}
