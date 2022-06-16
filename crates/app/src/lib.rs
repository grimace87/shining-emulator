
use bevy::{
    prelude::*,
    window::WindowResolution
};
use emulator::{Rom, Emulator, DummyAudioController};
use std::sync::{Arc, Mutex};

const ROM_NAME: &str = "test_rom.gb";

#[derive(Component)]
struct EmulatorComponent {
    emulator: Arc<Mutex<Emulator<DummyAudioController>>>
}

impl EmulatorComponent {
    pub fn new(emulator: Emulator<DummyAudioController>) -> Self {
        Self { emulator: Arc::new(Mutex::new(emulator)) }
    }
}

#[inline]
pub fn run_app<T>(platform_plugin: T) where T: Plugin {
    let mut app = App::new();
    app
        .insert_resource(ClearColor(Color::rgb(0.2, 0.2, 0.2)))
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Emulator".to_owned(),
                resizable: false,
                resolution: WindowResolution::default(),
                ..default()
            }),
            ..default()
        }))
        .add_plugin(platform_plugin)
        .add_startup_system(setup)
        .add_system(run_updates)
        .run();
}

fn setup(mut commands: Commands) {

    // Open ROM
    let file_data = files::local_rom_data(ROM_NAME)
        .unwrap();
    let rom = Rom::new_from_data(file_data);

    // Spawn the emulator
    let emulator = emulator::Emulator::load_rom(rom);
    commands.spawn(EmulatorComponent::new(emulator));
}

fn run_updates(mut query: Query<&EmulatorComponent>, time: Res<Time>) {

    let time_step = (time.delta_seconds_f64() * 1000.0) as u64;
    println!("Running time step of {}ms", time_step);

    let emulator = query.get_single_mut().unwrap();
    let mut lock = emulator.emulator.lock().unwrap();
    lock.do_work(time_step);
}
