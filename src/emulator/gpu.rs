
enum Mode {
    HBlank,
    VBlank,
    ScanOam,
    ScanVram
}

pub struct Gpu {
    clock_factor: u32,
    time_in_current_mode: u32,
    blanked_screen: bool,
    needing_clear: bool,
    mode: Mode,
    tile_set: Vec<u32>
}

impl Gpu {

    pub fn new() -> Self {
        let mut gpu = Self {
            clock_factor: 1,
            time_in_current_mode: 0,
            blanked_screen: false,
            needing_clear: false,
            mode: Mode::VBlank,
            tile_set: vec![2 * 384 * 8 * 8] // 2 VRAM banks, 384 tiles, 8 rows, 8 pixels per row
        };

        // Return object ready to run
        gpu.reset();
        gpu
    }

    pub fn reset(&mut self) {
    }
}
