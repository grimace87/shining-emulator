
mod filter;

enum Mode {
    HBlank,
    VBlank,
    ScanOam,
    ScanVram
}

pub struct Gpu {
    pub clock_factor: u32,
    time_in_current_mode: u32,
    blanked_screen: bool,
    needing_clear: bool,
    mode: Mode,
    pub tile_set: Vec<u32>,
    cgb_bg_pal_index: u32,
    cgb_obj_pal_index: u32
}

impl Gpu {

    pub fn new() -> Self {
        let mut gpu = Self {
            clock_factor: 1,
            time_in_current_mode: 0,
            blanked_screen: false,
            needing_clear: false,
            mode: Mode::VBlank,
            tile_set: vec![2 * 384 * 8 * 8], // 2 VRAM banks, 384 tiles, 8 rows, 8 pixels per row
            cgb_bg_pal_index: 0,
            cgb_obj_pal_index: 0
        };

        // Return object ready to run
        gpu.reset();
        gpu
    }

    pub fn reset(&mut self) {

        self.clock_factor = 1;
        self.mode = Mode::ScanOam;
        self.time_in_current_mode = 0;
        self.blanked_screen = false;
        self.cgb_bg_pal_index = 0;
        self.cgb_obj_pal_index = 0;
    }

    pub fn emulate_clock_cycles(&mut self, clock_cycles: i64) {
        // Double CPU speed mode affects instruction rate but should not affect GPU speed
        self.time_in_current_mode += (clock_cycles / (self.clock_factor as i64)) as u32;
        match self.mode {
            Mode::HBlank => {
                // WRITE THESE BITS
                // THEN NEED TO IMPLEMENT CPU OP CODES
                // AND ALSO THE FRAME MANAGER STUFF BELOW
            },
            Mode::VBlank => {

            },
            Mode::ScanOam => {

            },
            Mode::ScanVram => {

            }
        }
    }

    pub fn is_screen_blanked(&self) -> bool {
        self.blanked_screen
    }

    pub fn on_display_disabled(&mut self) {
        if self.blanked_screen {
            return;
        }
        self.time_in_current_mode = 0;
        self.mode = Mode::ScanOam;
        self.blanked_screen = true;

        // Mark any started frames as complete - probably won't do much
        // while frame_manager.frame_is_in_progress() {
        //     frame_manager.finish_current_frame();
        // }
    }
}
