
mod filter;

use crate::mem::MemoryMap;

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
    tile_set: Vec<u32>,
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

    pub fn emulate_clock_cycles<M: MemoryMap>(&mut self, clock_cycles: i64, mem_bus: &mut M) {
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

    pub fn on_display_disabled<M: MemoryMap>(&mut self, mem_bus: &mut M) {
        if self.blanked_screen {
            return;
        }
        mem_bus.set_oam_protection(false);
        mem_bus.set_vram_protection(false);
        mem_bus.write_address(0xff44, 0);
        self.time_in_current_mode = 0;
        self.mode = Mode::ScanOam;
        self.blanked_screen = true;

        // Mark any started frames as complete - probably won't do much
        // while frame_manager.frame_is_in_progress() {
        //     frame_manager.finish_current_frame();
        // }
    }
}
