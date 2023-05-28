
use crate::audio::AudioController;
use crate::external_ram::{Sram, ExternalRam};
use crate::input::Input;
use crate::rom::{Rom, Mbc};
use crate::sgb::Sgb;

use std::num::Wrapping;

const CLOCK_MULTIPLIERS: [i64; 21] = [  1,  1,  1, 1, 1, 2, 1, 4, 2, 4, 1, 5, 3, 7, 2, 5, 3, 5, 8, 12, 20 ];
const CLOCK_DIVISORS: [i64; 21] =    [ 20, 12,  8, 5, 3, 5, 2, 7, 3, 5, 1, 4, 2, 4, 1, 2, 1, 1, 1,  1,  1 ];

const GB_FREQ: u64 = 4194304;
const SGB_FREQ: u64 = 4295454;
const CGB_FREQ: u64 = 8400000;

#[derive(Eq, PartialEq, Copy, Clone)]
pub enum CpuType {
    Dmg,
    Sgb,
    Cgb
}

#[derive(Eq, PartialEq)]
enum Mode {
    Running,
    Halted,
    Stopped
}

#[repr(u8)]
enum GpuMode {
    HBlank = 0,
    VBlank = 1,
    ScanOam = 2,
    ScanVram = 3
}

pub struct Emulator<A: AudioController> {

    // Emulator
    sgb: Sgb,
    audio: A,
    input: Input,
    accumulated_clocks: u64,
    clock_multiply: i64,
    clock_divide: i64,
    current_clock_multiplier_combo: usize,

    // Mem bus
    pub rom: Rom,
    pub wram: Vec<Wrapping<u8>>,
    pub vram: Vec<Wrapping<u8>>,
    pub sram: Option<Sram>,
    pub oam: Vec<Wrapping<u8>>,
    pub io_ports: Vec<Wrapping<u8>>,
    vram_protected: bool,
    oam_protected: bool,
    rom_bank_offset: Wrapping<usize>,
    wram_bank_offset: Wrapping<usize>,
    vram_bank_offset: Wrapping<usize>,
    rom_mbc_mode: u32,

    // CPU
    cpu_type: CpuType,
    pc: Wrapping<usize>,
    sp: Wrapping<usize>,
    a: Wrapping<u8>,
    f: Wrapping<u8>,
    b: Wrapping<u8>,
    c: Wrapping<u8>,
    d: Wrapping<u8>,
    e: Wrapping<u8>,
    h: Wrapping<u8>,
    l: Wrapping<u8>,
    mode: Mode,
    is_running: bool,
    is_paused: bool,
    clock_frequency: u64,
    ime: bool,
    divider_count: u32,
    timer_count: u32,
    timer_inc_time: u32,
    timer_running: bool,
    last_ly_compare: u32,
    current_key_dir: u8,
    current_key_but: u8,
    key_state_changed: bool,

    // GPU
    gpu_clock_factor: u32,
    time_in_current_gpu_mode: Wrapping<u32>,
    blanked_screen: bool,
    needing_clear: bool,
    gpu_mode: GpuMode,
    tile_set: Vec<u32>,
    cgb_bg_pal_index: u32,
    cgb_obj_pal_index: u32,

    // Serial
    serial_request: bool,
    serial_is_transferring: bool,
    serial_clock_is_external: bool,
    serial_timer: i32
}

impl<A: AudioController> Emulator<A> {

    pub fn load_rom(rom: Rom) -> Self {
        let sram = rom.get_sram();
        let cpu_type = rom.get_preferred_hardware_type();
        let mut emulator = Self {

            sgb: Sgb::new(),
            audio: A::new(),
            input: Input::new(),
            accumulated_clocks: 0,
            clock_multiply: 1,
            clock_divide: 1,
            current_clock_multiplier_combo: 10,

            rom,
            wram: vec![Wrapping(0); 8 * 4096],
            vram: vec![Wrapping(0); 2 * 8192],
            sram,
            oam: vec![Wrapping(0); 160],
            io_ports: vec![Wrapping(0); 256],
            vram_protected: false,
            oam_protected: false,
            rom_bank_offset: Wrapping(0),
            wram_bank_offset: Wrapping(0),
            vram_bank_offset: Wrapping(0),
            rom_mbc_mode: 0,

            cpu_type,
            pc: Wrapping(0),
            sp: Wrapping(0),
            a: Wrapping(0),
            f: Wrapping(0),
            b: Wrapping(0),
            c: Wrapping(0),
            d: Wrapping(0),
            e: Wrapping(0),
            h: Wrapping(0),
            l: Wrapping(0),
            mode: Mode::Stopped,
            is_running: false,
            is_paused: false,
            clock_frequency: GB_FREQ,
            ime: false,
            divider_count: 0,
            timer_count: 0,
            timer_inc_time: 0,
            timer_running: false,
            last_ly_compare: 0,
            current_key_dir: 0x0f,
            current_key_but: 0x0f,
            key_state_changed: false,

            gpu_clock_factor: 1,
            time_in_current_gpu_mode: Wrapping(0),
            blanked_screen: false,
            needing_clear: false,
            gpu_mode: GpuMode::VBlank,
            tile_set: vec![0; 2 * 384 * 8 * 8], // 2 VRAM banks, 384 tiles, 8 rows, 8 pixels per row
            cgb_bg_pal_index: 0,
            cgb_obj_pal_index: 0,

            serial_request: false,
            serial_is_transferring: false,
            serial_clock_is_external: false,
            serial_timer: 0
        };
        emulator.reset();
        emulator
    }

    pub fn reset(&mut self) {

        let cpu_type = self.rom.get_preferred_hardware_type();

        // Reset component states
        self.sgb.reset();
        self.audio.reset(self.clock_frequency);
        self.input.reset();

        // Reset clock multipliers
        self.current_clock_multiplier_combo = 10;
        self.clock_multiply = 1;
        self.clock_divide = 1;

        if let Some(sram) = &mut self.sram {
            sram.reset();
        }

        // Clear block memory
        self.wram.fill(Wrapping(0));
        self.vram.fill(Wrapping(0));
        self.oam.fill(Wrapping(0));
        self.io_ports.fill(Wrapping(0));

        // Set various state
        self.vram_protected = false;
        self.oam_protected = false;
        self.rom_bank_offset = Wrapping(0x4000);
        self.wram_bank_offset = Wrapping(0x1000);
        self.vram_bank_offset = Wrapping(0x0000);
        self.rom_mbc_mode = 0;

        // Initialise IO ports
        self.io_ports[5] = Wrapping(0x00);
        self.io_ports[6] = Wrapping(0x00);
        self.io_ports[7] = Wrapping(0x00);
        self.io_ports[16] = Wrapping(0x80);
        self.io_ports[17] = Wrapping(0xbf);
        self.io_ports[18] = Wrapping(0xf3);
        self.io_ports[20] = Wrapping(0xbf);
        self.io_ports[22] = Wrapping(0x3f);
        self.io_ports[23] = Wrapping(0x00);
        self.io_ports[25] = Wrapping(0xbf);
        self.io_ports[26] = Wrapping(0x7f);
        self.io_ports[27] = Wrapping(0xff);
        self.io_ports[28] = Wrapping(0x9f);
        self.io_ports[30] = Wrapping(0xbf);
        self.io_ports[32] = Wrapping(0xff);
        self.io_ports[33] = Wrapping(0x00);
        self.io_ports[34] = Wrapping(0x00);
        self.io_ports[35] = Wrapping(0xbf);
        self.io_ports[36] = Wrapping(0x77);
        self.io_ports[37] = Wrapping(0xf3);
        self.io_ports[64] = Wrapping(0x91);
        self.io_ports[66] = Wrapping(0x00);
        self.io_ports[67] = Wrapping(0x00);
        self.io_ports[69] = Wrapping(0x00);
        self.io_ports[71] = Wrapping(0xfc);
        self.io_ports[72] = Wrapping(0xff);
        self.io_ports[73] = Wrapping(0xff);
        self.io_ports[74] = Wrapping(0x00);
        self.io_ports[75] = Wrapping(0x00);
        self.io_ports[85] = Wrapping(0xff);
        self.io_ports[255] = Wrapping(0x00);

        // Conditional state
        if cpu_type == CpuType::Sgb {
            self.io_ports[38] = Wrapping(0xf0);
        } else {
            self.io_ports[38] = Wrapping(0xf1);
        }

        self.is_running = true;
        self.is_paused = false;
        self.ime = false;
        self.mode = Mode::Running;
        self.divider_count = 0;
        self.timer_count = 0;
        self.timer_inc_time = 1024;
        self.timer_running = false;
        self.last_ly_compare = 1;
        self.key_state_changed = false;

        self.gpu_clock_factor = 1;
        self.gpu_mode = GpuMode::ScanOam;
        self.time_in_current_gpu_mode = Wrapping(0);
        self.blanked_screen = false;
        self.cgb_bg_pal_index = 0;
        self.cgb_obj_pal_index = 0;

        self.serial_request = false;
        self.serial_is_transferring = false;
        self.serial_clock_is_external = false;
        self.serial_timer = 0;

        // Set various state
        self.pc.0 = 0x0100;
        self.sp.0 = 0xfffe;
        self.f.0 = 0xb0;
        self.b.0 = 0x00;
        self.c.0 = 0x13;
        self.d.0 = 0x00;
        self.e.0 = 0xd8;
        self.h.0 = 0x01;
        self.l.0 = 0x4d;

        // Hardware-dependent state
        match cpu_type {
            CpuType::Dmg => {
                self.a.0 = 0x01;
                self.clock_frequency = GB_FREQ;
            },
            CpuType::Sgb => {
                self.a.0 = 0x11;
                self.clock_frequency = SGB_FREQ;
            },
            CpuType::Cgb => {
                self.a.0 = 0x11;
                self.clock_frequency = GB_FREQ;
            }
        }
    }

    pub fn start(&mut self) {

    }

    pub fn stop(&mut self) {

    }

    pub fn speed_up(&mut self) {
        if self.current_clock_multiplier_combo < 20 {
            self.current_clock_multiplier_combo += 1;
            self.clock_multiply = CLOCK_MULTIPLIERS[self.current_clock_multiplier_combo];
            self.clock_divide = CLOCK_DIVISORS[self.current_clock_multiplier_combo];
        }
    }

    pub fn slow_down(&mut self) {
        if self.current_clock_multiplier_combo > 0 {
            self.current_clock_multiplier_combo -= 1;
            self.clock_multiply = CLOCK_MULTIPLIERS[self.current_clock_multiplier_combo];
            self.clock_divide = CLOCK_DIVISORS[self.current_clock_multiplier_combo];
        }
    }

    pub fn do_work(&mut self, time_diff_millis: u64) {

        let cpu_frequency = self.clock_frequency as i64;
        let adjusted_frequency =
            (cpu_frequency * self.clock_multiply / self.clock_divide) as f64;
        self.accumulated_clocks += (time_diff_millis as f64 * 0.001 * adjusted_frequency) as u64;
        let approx_multiplier = (self.clock_multiply / self.clock_divide + 1) as u64;
        let clocks_cap = 1000000 * approx_multiplier;
        if self.accumulated_clocks > clocks_cap {
            self.accumulated_clocks = clocks_cap;
        }

        self.set_input_values(self.input.key_dir, self.input.key_but);

        let consumed_clocks = self.emulate_clock_cycles();
    }

    pub fn set_input_values(&mut self, key_dir: u8, key_but: u8) {
        if key_dir != self.current_key_dir || key_but != self.current_key_but {
            self.key_state_changed = true;
        }
        self.current_key_dir = key_dir;
        self.current_key_but = key_but;
    }

    fn on_invalid_instruction(&mut self, instruction: u8) -> u64 {
        self.is_running = false;
        0
    }

    fn on_display_disabled(&mut self) {
        if self.blanked_screen {
            return;
        }
        self.time_in_current_gpu_mode = Wrapping(0);
        self.gpu_mode = GpuMode::ScanOam;
        self.blanked_screen = true;

        // Mark any started frames as complete - probably won't do much
        // while frame_manager.frame_is_in_progress() {
        //     frame_manager.finish_current_frame();
        // }
    }

    pub fn emulate_clock_cycles(&mut self) -> i64 {

        let accumulated_clocks = self.accumulated_clocks as i64;
        if !self.is_running || self.is_paused {
            // Consume all clocks
            return accumulated_clocks;
        }

        if self.key_state_changed {
            self.key_state_changed = false;

            // Adjust value in keypad register
            let upper_bits = self.read_address(Wrapping(0xff00)).0 & 0x30;
            if upper_bits == 0x20 {
                self.perform_and(Wrapping(0xff00), Wrapping(0xf0));
                self.perform_or(Wrapping(0xff00), Wrapping(self.current_key_dir));
            } else if upper_bits == 0x10 {
                self.perform_and(Wrapping(0xff00), Wrapping(0xf0));
                self.perform_or(Wrapping(0xff00), Wrapping(self.current_key_but));
            }

            // Set interrupt request flag
            self.perform_or(Wrapping(0xff0f), Wrapping(0x10));
        }

        let mut remaining_clocks = accumulated_clocks;
        while remaining_clocks > 0 {
            let progress = self.emulate_next_step() as i64;
            if progress <= 32 {
                let display_enabled = (self.read_address(Wrapping(0xff40)).0 & 0x80) != 0;
                if display_enabled {
                    self.emulate_gpu(progress);
                } else {
                    if !self.blanked_screen {
                        self.set_oam_protection(false);
                        self.set_vram_protection(false);
                        self.write_address(Wrapping(0xff44), Wrapping(0));
                        self.on_display_disabled();
                    }
                }
            }
            remaining_clocks -= progress;
        }
        accumulated_clocks - remaining_clocks
    }

    #[inline]
    fn emulate_gpu(&mut self, clock_cycles: i64) {
        // Double CPU speed mode affects instruction rate but should not affect GPU speed
        self.time_in_current_gpu_mode += (clock_cycles / (self.gpu_clock_factor as i64)) as u32;
        match self.gpu_mode {
            GpuMode::HBlank => {
                // Spends 204 cycles here, then moves to next line. After 144th hblank, move to vblank.
                if self.time_in_current_gpu_mode.0 < 204 {
                    return;
                }
                self.time_in_current_gpu_mode -= 204;
                self.io_ports[0x44] += 1;
                if self.io_ports[0x44].0 == 144 {
                    self.gpu_mode = GpuMode::VBlank;
                    self.io_ports[0x41] &= 0xfc;
                    self.io_ports[0x41] |= GpuMode::VBlank as u8;
                    // Set interrupt request for VBLANK
                    self.io_ports[0x0f] |= 0x01;
                    self.oam_protected = false;
                    self.vram_protected = false;
                    if (self.io_ports[0x41].0 & 0x10) != 0x00 {
                        // Request status int if condition met
                        self.io_ports[0x0f] |= 0x02;
                    }
                    // This is where stuff can be drawn - on the beginning of the vblank
                    if !self.sgb.freeze_screen {
                        // if (frameManager.frameIsInProgress()) {
                        //     auto frameBuffer = frameManager.getInProgressFrameBuffer();
                        //     if ((frameBuffer != nullptr) && romProperties.sgbFlag) {
                        //         sgb.colouriseFrame(frameBuffer);
                        //     }
                        //     frameManager.finishCurrentFrame();
                        // }
                    }
                } else {
                    self.gpu_mode = GpuMode::ScanOam;
                    self.io_ports[0x41] &= 0xfc;
                    self.io_ports[0x41] |= GpuMode::ScanOam as u8;
                    self.oam_protected = true;
                    self.vram_protected = false;
                    if (self.io_ports[0x41].0 & 0x20) != 0x00 {
                        // Request status int if condition met
                        self.io_ports[0x0f] |= 0x02;
                    }
                }
            },
            GpuMode::VBlank => {
                if self.time_in_current_gpu_mode.0 < 456 {
                    return;
                }
                // 10 of these lines in vblank
                self.time_in_current_gpu_mode -= 456;
                self.io_ports[0x44] += 1;
                if self.io_ports[0x44].0 >= 154 {
                    self.gpu_mode = GpuMode::ScanOam;
                    self.io_ports[0x41] &= 0xfc;
                    self.io_ports[0x41] |= GpuMode::ScanOam as u8;
                    self.io_ports[0x44] = Wrapping(0);
                    self.oam_protected = true;
                    self.vram_protected = false;
                    if (self.io_ports[0x41].0 & 0x20) != 0x00 {
                        // Request status int if condition met
                        self.io_ports[0x0f] |= 0x02;
                    }

                    // LCD starting at top of frame, ready a frame buffer if available
                    // frameManager.beginNewFrame();
                }
            },
            GpuMode::ScanOam => {
                if self.time_in_current_gpu_mode.0 < 80 {
                    return;
                }
                self.time_in_current_gpu_mode -= 80;
                self.gpu_mode = GpuMode::ScanVram;
                self.io_ports[0x41] &= 0xfc;
                self.io_ports[0x41] |= GpuMode::ScanVram as u8;
                self.oam_protected = true;
                self.vram_protected = true;
            },
            GpuMode::ScanVram => {
                if self.time_in_current_gpu_mode.0 < 172 {
                    return;
                }
                self.time_in_current_gpu_mode -= 172;
                self.gpu_mode = GpuMode::HBlank;
                self.io_ports[0x41] &= 0xfc;
                self.io_ports[0x41] |= GpuMode::HBlank as u8;
                self.oam_protected = false;
                self.vram_protected = false;
                if (self.io_ports[0x41].0 & 0x08) != 0x00 {
                    // Request status int if condition met
                    self.io_ports[0x0f] |= 0x02;
                }
                // Run DMA if applicable
                if self.io_ports[0x55].0 < 0xff {
                    // H-blank DMA currently active
                    let mut src_addr = Wrapping(((self.io_ports[0x51].0 as usize) << 8) + self.io_ports[0x52].0 as usize); // DMA source
                    // Don't do transfers within VRAM or from these other addresses either
                    if (src_addr.0 & 0xe000) != 0x8000 && src_addr.0 < 0xe000 {
                        let mut dst_addr = Wrapping(((self.io_ports[0x53].0 as usize) << 8) + self.io_ports[0x54].0 as usize + 0x8000); // DMA destination
                        for count in 0..16 {
                            self.write_address(dst_addr, self.read_address(src_addr));
                            src_addr += 1;
                            dst_addr = (dst_addr + Wrapping(1)) & Wrapping(0x9fff); // Keep it within VRAM
                        }
                    }

                    // TODO - what is this?
                    //if (ClockFreq == GBC_FREQ) clocks_acc -= 64;
                    //else clocks_acc -= 32;
                    self.io_ports[0x55] -= 1;
                    if self.io_ports[0x55].0 < 0x80 {
                        // End the DMA
                        self.io_ports[0x55].0 = 0xff;
                    }
                }

                // Process current line's graphics
                // if (frameManager.frameIsInProgress()) {
                //     (*this.*readLine)(frameManager.getInProgressFrameBuffer());
                // }
            }
        }
    }

    /// Simulate one instruction, returning the number of clock cycles progressed.
    #[inline]
    fn emulate_next_step(&mut self) -> u64 {
        let clocks_passed_by_instruction = self.perform_op();
        self.pc &= 0xffff;

        // Check for interrupts
        let cpu_halted = self.mode == Mode::Halted;
        if self.ime || cpu_halted {
            self.process_interrupts(cpu_halted);
        }

        // While CPU is in stop mode, nothing much still runs
        if self.mode == Mode::Stopped {
            if self.switch_running_speed() {
                self.mode = Mode::Running;
                return clocks_passed_by_instruction + 131072;
            }
            return clocks_passed_by_instruction;
        }

        // Permanent compare of LY and LYC
        let display_enabled = (self.read_address(Wrapping(0xff40)) & Wrapping(0x80)).0 != 0;
        if display_enabled && self.read_address(Wrapping(0xff44)) == self.read_address(Wrapping(0xff45)) {

            // Set coincidence flag
            self.perform_or(Wrapping(0xff41), Wrapping(0x04));

            // Request interrupt if his signal goes low to high
            if (self.read_address(Wrapping(0xff41)).0 & 0x40) != 0 && self.last_ly_compare == 0 {
                self.perform_or(Wrapping(0xff0f), Wrapping(0x02));
            }
            self.last_ly_compare = 1;
        } else {
            self.perform_and(Wrapping(0xff41), Wrapping(0xfb));
            self.last_ly_compare = 0;
        }

        // Handling of timers
        self.divider_count += clocks_passed_by_instruction as u32;
        if self.divider_count >= 256 {
            self.divider_count -= 256;
            self.write_address(Wrapping(0xff04), self.read_address(Wrapping(0xff04)) + Wrapping(1));
        }
        if self.timer_running {
            self.timer_count += 1;
            if self.timer_count >= self.timer_inc_time {
                self.timer_count -= self.timer_inc_time;
                let new_register = self.read_address(Wrapping(0xff05)) + Wrapping(1);
                self.write_address(Wrapping(0xff05), new_register);
                if new_register.0 == 0 {
                    self.write_address(Wrapping(0xff05), self.read_address(Wrapping(0xff06)));
                    self.perform_or(Wrapping(0xff0f), Wrapping(0x04));
                }
            }
        }

        // Handle audio
        self.audio.simulate(clocks_passed_by_instruction / (self.gpu_clock_factor as u64));

        // Handle serial port timeout
        if self.serial_is_transferring {
            if !self.serial_clock_is_external {
                self.serial_timer -= clocks_passed_by_instruction as i32;
                if self.serial_timer <= 0 {
                    self.serial_is_transferring = false;
                    self.io_ports[0x02] &= 0x03;
                    self.io_ports[0x0f] |= 0x08;
                    self.io_ports[0x01] = Wrapping(0xff);
                }
            } else {
                if self.serial_timer == 1 {
                    self.serial_timer = 0;
                }
            }
        }

        clocks_passed_by_instruction
    }

    #[inline]
    fn process_interrupts(&mut self, cpu_halted: bool) {

        let triggered_interrupts = self.read_address(Wrapping(0xffff)).0 & self.read_address(Wrapping(0xff0f)).0 & 0x1f;
        if triggered_interrupts == 0 {
            return;
        }

        let mut to_address = self.pc;
        if (triggered_interrupts & 0x01) != 0 {
            // V-blank
            self.perform_and(Wrapping(0xff0f), Wrapping(0x1e));
            to_address.0 = 0x0040;
        } else if (triggered_interrupts & 0x02) != 0 {
            // LCD stat
            self.perform_and(Wrapping(0xff0f), Wrapping(0x1d));
            to_address.0 = 0x0048;
        } else if (triggered_interrupts & 0x04) != 0 {
            // Timer
            self.perform_and(Wrapping(0xff0f), Wrapping(0x1b));
            to_address.0 = 0x0050;
        } else if (triggered_interrupts & 0x08) != 0 {
            // Serial
            self.perform_and(Wrapping(0xff0f), Wrapping(0x17));
            to_address.0 = 0x0058;
        } else if (triggered_interrupts & 0x10) != 0 {
            // Joypad
            self.perform_and(Wrapping(0xff0f), Wrapping(0x0f));
            to_address.0 = 0x0060;
        }

        // Unless halted with IME unset, push PC onto stack and go to interrupt handler address
        if !cpu_halted || self.ime {
            self.sp -= 2;
            self.write_address_16(
                self.sp, (Wrapping((self.pc.0 & 0xff) as u8), Wrapping((self.pc >> 8).0 as u8)));
            self.pc = to_address;
        }
        self.mode = Mode::Running;
        self.ime = false;
    }

    #[inline]
    fn get_address_hl(&self) -> Wrapping<usize> {
        Wrapping(((self.h.0 as usize) << 8) + self.l.0 as usize)
    }

    #[inline]
    fn read_byte_hl(&self) -> Wrapping<u8> {
        self.read_address(Wrapping(((self.h.0 as usize) << 8) + self.l.0 as usize))
    }

    #[inline]
    fn write_byte_hl(&mut self, byte: Wrapping<u8>) {
        self.write_address(Wrapping(((self.h.0 as usize) << 8) + self.l.0 as usize), byte);
    }

    #[inline]
    fn set_z_on_zero(&mut self, test_value: u8) {
        if test_value == 0x00 {
            self.f |= 0x80;
        }
    }

    #[inline]
    fn set_z_on_condition(&mut self, test: bool) {
        if test {
            self.f |= 0x80;
        }
    }

    #[inline]
    fn set_h_on_zero(&mut self, test_value: u8) {
        if test_value == 0x00 {
            self.f |= 0x20;
        }
    }

    #[inline]
    fn set_h_on_condition(&mut self, test: bool) {
        if test {
            self.f |= 0x20;
        }
    }

    #[inline]
    fn set_c_on_condition(&mut self, test: bool) {
        if test {
            self.f |= 0x10;
        }
    }

    #[inline]
    fn switch_running_speed(&mut self) -> bool {
        let speed_change_requested = self.cpu_type == CpuType::Cgb && self.read_address(Wrapping(0xff4d)).0 == 0x01;
        if speed_change_requested {
            // Speed change was requested in CGB mode
            self.perform_and(Wrapping(0xff4d), Wrapping(0x80));
            let new_byte = self.read_address(Wrapping(0xff4d));
            if new_byte.0 == 0x00 {
                self.write_address(Wrapping(0xff4d), Wrapping(0x80));
                self.clock_frequency = CGB_FREQ;
                self.gpu_clock_factor = 2;
                return true;
            } else {
                self.write_address(Wrapping(0xff4d), Wrapping(0x00));
                self.clock_frequency = GB_FREQ;
                self.gpu_clock_factor = 1;
                return true;
            }
        }
        false
    }

    #[inline]
    fn perform_op(&mut self) -> u64 {
        let instr = self.read_address(self.pc).0;
        match instr {
            0x00 => { // nop
                self.pc += 1;
                4
            },
            0x01 => { // ld BC, nn
                self.b = self.read_address(self.pc + Wrapping(2));
                self.c = self.read_address(self.pc + Wrapping(1));
                self.pc += 3;
                12
            },
            0x02 => { // ld (BC), A
                self.write_address(Wrapping(((self.b.0 as usize) << 8) + self.c.0 as usize), self.a);
                self.pc += 1;
                8
            },
            0x03 => { // inc BC
                self.c += 1;
                if self.c.0 == 0x00 {
                    self.b += 1;
                }
                self.pc += 1;
                8
            },
            0x04 => { // inc B
                self.f &= 0x10;
                self.b += 0x01;
                self.set_z_on_zero(self.b.0);
                self.set_h_on_zero(self.b.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x05 => { // dec B
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.b.0 & 0x0f);
                self.b -= 0x01;
                self.set_z_on_zero(self.b.0);
                self.pc += 1;
                4
            },
            0x06 => { // ld B, n
                self.b = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x07 => { // rlc A (rotate bit 7 to bit 0, and copy bit 7 to carry flag)
                let temp_byte = self.a.0 & 0x80; // True if bit 7 is set
                self.f.0 = 0x00;
                self.a = self.a << 1;
                if temp_byte != 0 {
                    self.f |= 0x10; // Set carry
                    self.a |= 0x01;
                }
                self.pc += 1;
                4
            },
            0x08 => { // ld (nn), SP
                self.write_address_16(
                    Wrapping(((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize),
                    (Wrapping((self.sp.0 & 0x00ff) as u8), Wrapping(((self.sp.0 >> 8) & 0x00ff) as u8))
                );
                self.pc += 3;
                20
            },
            0x09 => { // add HL, BC
                self.f &= 0x80;
                self.l += self.c;
                if self.l < self.c {
                    self.h += 1;
                    self.set_c_on_condition(self.h.0 == 0x00);
                    self.set_h_on_zero(self.h.0 & 0x0f);
                }
                self.h += self.b;
                self.set_c_on_condition(self.h < self.b);
                self.set_h_on_condition((self.h.0 & 0x0f) < (self.b.0 & 0x0f));
                self.pc += 1;
                8
            },
            0x0a => { // ld A, (BC)
                self.a = self.read_address(Wrapping(((self.b.0 as usize) << 8) + self.c.0 as usize));
                self.pc += 1;
                8
            },
            0x0b => { // dec BC
                if self.c.0 == 0x00 {
                    self.b -= 1;
                }
                self.c -= 1;
                self.pc += 1;
                8
            },
            0x0c => { // inc C
                self.f &= 0x10;
                self.c += 0x01;
                self.set_z_on_zero(self.c.0);
                self.set_h_on_zero(self.c.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x0d => { // dec C
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.c.0 & 0x0f);
                self.c -= 0x01;
                self.set_z_on_zero(self.c.0);
                self.pc += 1;
                4
            },
            0x0e => { // ld C, n
                self.c = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x0f => { // rrc A (8-bit rotation right - bit 0 is moved to carry also)
                let temp_byte = self.a.0 & 0x01;
                self.f.0 = 0x00;
                self.a = self.a >> 1;
                self.a &= 0x7f; // Clear msb in sign bit preserved by compiler
                if temp_byte != 0 {
                    self.f.0 = 0x10;
                    self.a |= 0x80;
                }
                self.pc += 1;
                4
            },
            0x10 => { // stop
                self.mode = Mode::Stopped;
                self.pc += 1;
                4
            },
            0x11 => { // ld DE, nn
                self.d = self.read_address(self.pc + Wrapping(2));
                self.e = self.read_address(self.pc + Wrapping(1));
                self.pc += 3;
                12
            },
            0x12 => { // ld (DE), A
                self.write_address(Wrapping(((self.d.0 as usize) << 8) + self.e.0 as usize), self.a);
                self.pc += 1;
                8
            },
            0x13 => { // inc DE
                self.e += 1;
                if self.e.0 == 0x00 {
                    self.d += 1;
                }
                self.pc += 1;
                8
            },
            0x14 => { // inc D
                self.f &= 0x10;
                self.d += 0x01;
                self.set_z_on_zero(self.d.0);
                self.set_h_on_zero(self.d.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x15 => { // dec D
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.d.0 & 0x0f);
                self.d -= 0x01;
                self.set_z_on_zero(self.d.0);
                self.pc += 1;
                4
            },
            0x16 => { // ld D, n
                self.d = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x17 => { // rl A (rotate carry bit to bit 0 of A)
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x00;
                self.set_c_on_condition((self.a.0 & 0x80) != 0); // Copy bit 7 to carry bit
                self.a = self.a << 1;
                if carry_was_set {
                    self.a |= 0x01; // Copy carry flag to bit 0
                }
                self.pc += 1;
                4
            },
            0x18 => { // jr d
                let msb = self.read_address(self.pc + Wrapping(1)).0;
                if msb >= 0x80 {
                    self.pc -= 256 - msb as usize;
                } else {
                    self.pc += msb as usize;
                }
                self.pc += 2;
                12
            },
            0x19 => { // add HL, DE
                self.f &= 0x80;
                self.l += self.e;
                if self.l < self.e {
                    self.h += 1;
                    self.set_c_on_condition(self.h.0 == 0x00);
                    self.set_h_on_zero(self.h.0 & 0x0f);
                }
                self.h += self.d;
                self.set_c_on_condition(self.h < self.d);
                self.set_h_on_condition((self.h.0 & 0x0f) < (self.d.0 & 0x0f));
                self.pc += 1;
                8
            },
            0x1a => { // ld A, (DE)
                self.a = self.read_address(Wrapping(((self.d.0 as usize) << 8) + self.e.0 as usize));
                self.pc += 1;
                8
            },
            0x1b => { // dec DE
                if self.e.0 == 0x00 {
                    self.d -= 1;
                }
                self.e -= 1;
                self.pc += 1;
                8
            },
            0x1c => { // inc E
                self.f &= 0x10;
                self.e += 0x01;
                self.set_z_on_zero(self.e.0);
                self.set_h_on_zero(self.e.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x1d => { // dec E
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.e.0 & 0x0f);
                self.e -= 0x01;
                self.set_z_on_zero(self.e.0);
                self.pc += 1;
                4
            },
            0x1e => { // ld E, n
                self.e = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x1f => { // rr A (9-bit rotation right of A through carry)
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x00;
                self.set_c_on_condition((self.a.0 & 0x01) != 0x00);
                self.a = self.a >> 1;
                self.a.0 = self.a.0 & 0x7f;
                if carry_was_set {
                    self.a |= 0x80;
                }
                self.pc += 1;
                4
            },
            0x20 => { // jr NZ, d
                if (self.f.0 & 0x80) != 0 {
                    self.pc += 2;
                    8
                } else {
                    let msb = self.read_address(self.pc + Wrapping(1)).0;
                    if msb >= 0x80 {
                        self.pc -= 256 - msb as usize;
                    } else {
                        self.pc += msb as usize;
                    }
                    self.pc += 2;
                    12
                }
            },
            0x21 => { // ld HL, nn
                self.h = self.read_address(self.pc + Wrapping(2));
                self.l = self.read_address(self.pc + Wrapping(1));
                self.pc += 3;
                12
            },
            0x22 => { // ldi (HL), A
                self.write_byte_hl(self.a);
                self.l += 1;
                if self.l.0 == 0x00 {
                    self.h += 1; // L overflowed into H
                }
                self.pc += 1;
                8
            },
            0x23 => { // inc HL
                self.l += 1;
                if self.l.0 == 0x00 {
                    self.h += 1;
                }
                self.pc += 1;
                8
            },
            0x24 => { // inc H
                self.f &= 0x10;
                self.h += 0x01;
                self.set_z_on_zero(self.h.0);
                self.set_h_on_zero(self.h.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x25 => { // dec H
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.h.0 & 0x0f);
                self.h -= 0x01;
                self.set_z_on_zero(self.h.0);
                self.pc += 1;
                4
            },
            0x26 => { // ld H, n
                self.h = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x27 => { // daa (Decimal Adjust Accumulator - do BCD correction)
                if (self.f.0 & 0x40) == 0x00 {
                    if ((self.a.0 & 0x0f) > 0x09) || ((self.f.0 & 0x20) != 0x00) { // If lower 4 bits are non-decimal or H is set, add 0x06
                        self.a += 0x06;
                    }
                    let carry_was_set = (self.f.0 & 0x10) != 0x00;
                    self.f &= 0x40; // Reset C, H and Z flags
                    if (self.a.0 > 0x9f) || carry_was_set { // If upper 4 bits are non-decimal or C was set, add 0x60
                        self.a += 0x60;
                        self.f |= 0x10; // Sets the C flag if this second addition was needed
                    }
                } else {
                    // TODO - Check why comment says "add" but actually we subtract
                    if ((self.a.0 & 0x0f) > 0x09) || ((self.f.0 & 0x20) != 0x00) { // If lower 4 bits are non-decimal or H is set, add 0x06
                        self.a -= 0x06;
                    }
                    let carry_was_set = (self.f.0 & 0x10) != 0x00;
                    self.f &= 0x40; // Reset C, H and Z flags
                    if (self.a.0 > 0x9f) || carry_was_set { // If upper 4 bits are non-decimal or C was set, add 0x60
                        self.a -= 0x60;
                        self.f |= 0x10; // Sets the C flag if this second addition was needed
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x28 => { // jr Z, d
                if (self.f.0 & 0x80) != 0x00 {
                    let msb = self.read_address(self.pc + Wrapping(1)).0;
                    if msb >= 0x80 {
                        self.pc -= 256 - msb as usize;
                    } else {
                        self.pc += msb as usize;
                    }
                    self.pc += 2;
                    12
                } else {
                    self.pc += 2;
                    8
                }
            },
            0x29 => { // add HL, HL
                self.f &= 0x80;
                self.set_c_on_condition((self.h.0 & 0x80) != 0x00);
                self.set_h_on_condition((self.h.0 & 0x08) != 0x00);
                if (self.l.0 & 0x80) != 0x00 {
                    self.h *= 2;
                    self.l *= 2;
                    self.h += 1;
                } else {
                    self.h *= 2;
                    self.l *= 2;
                }
                self.pc += 1;
                8
            },
            0x2a => { // ldi A, (HL)
                self.a = self.read_byte_hl();
                self.l += 1;
                if self.l.0 == 0x00 {
                    self.h += 1;
                }
                self.pc += 1;
                8
            },
            0x2b => { // dec HL
                if self.l.0 == 0x00 {
                    self.h -= 1;
                }
                self.l -= 1;
                self.pc += 1;
                8
            },
            0x2c => { // inc L
                self.f &= 0x10;
                self.l += 0x01;
                self.set_z_on_zero(self.l.0);
                self.set_h_on_zero(self.l.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x2d => { // dec L
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.l.0 & 0x0f);
                self.l -= 0x01;
                self.set_z_on_zero(self.l.0);
                self.pc += 1;
                4
            },
            0x2e => { // ld L, n
                self.l = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x2f => { // cpl A (complement - bitwise NOT)
                self.a = !self.a;
                self.f |= 0x60;
                self.pc += 1;
                4
            },
            0x30 => { // jr NC, d
                if (self.f.0 & 0x10) != 0x00 {
                    self.pc += 2;
                    8
                } else {
                    let msb = self.read_address(self.pc + Wrapping(1)).0;
                    if msb >= 0x80 {
                        self.pc -= 256 - msb as usize;
                    } else {
                        self.pc += msb as usize;
                    }
                    self.pc += 2;
                    12
                }
            },
            0x31 => { // ld SP, nn
                self.sp.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                self.pc += 3;
                12
            },
            0x32 => { // ldd (HL), A
                self.write_byte_hl(self.a);
                if self.l.0 == 0x00 {
                    self.h -= 1;
                }
                self.l -= 1;
                self.pc += 1;
                8
            },
            0x33 => { // inc SP
                self.sp += 1;
                self.sp &= 0xffff;
                self.pc += 1;
                8
            },
            0x34 => { // inc (HL)
                self.f &= 0x10;
                let temp_addr = self.get_address_hl();
                let temp_byte = self.read_address(temp_addr) + Wrapping(1);
                self.set_z_on_zero(temp_byte.0);
                self.set_h_on_zero(temp_byte.0 & 0x0f);
                self.write_address(temp_addr, temp_byte);
                self.pc += 1;
                12
            },
            0x35 => { // dec (HL)
                self.f &= 0x10;
                self.f |= 0x40;
                let temp_addr = self.get_address_hl();
                let mut temp_byte = self.read_address(temp_addr);
                self.set_h_on_zero(temp_byte.0 & 0x0f);
                temp_byte -= 1;
                self.set_z_on_zero(temp_byte.0);
                self.write_address(temp_addr, temp_byte);
                self.pc += 1;
                12
            },
            0x36 => { // ld (HL), n
                self.write_byte_hl(self.read_address(self.pc + Wrapping(1)));
                self.pc += 2;
                12
            },
            0x37 => { // SCF (set carry flag)
                self.f &= 0x80;
                self.f |= 0x10;
                self.pc += 1;
                4
            },
            0x38 => { // jr C, n
                if (self.f.0 & 0x10) != 0x00 {
                    let msb = self.read_address(self.pc + Wrapping(1)).0;
                    if msb >= 0x80 {
                        self.pc -= 256 - msb as usize;
                    } else {
                        self.pc += msb as usize;
                    }
                    self.pc += 2;
                    12
                } else {
                    self.pc += 2;
                    8
                }
            },
            0x39 => { // add HL, SP
                self.f &= 0x80;
                let mut temp_byte = (self.sp.0 & 0x00ff) as u8;
                self.l += temp_byte;
                if self.l.0 < temp_byte {
                    self.h += 1;
                }
                temp_byte = (self.sp.0 >> 8) as u8;
                self.h += temp_byte;
                self.set_c_on_condition(self.h.0 < temp_byte);
                temp_byte = temp_byte & 0x0f;
                self.set_h_on_condition((self.h.0 & 0x0f) < temp_byte);
                self.pc += 1;
                8
            },
            0x3a => { // ldd A, (HL)
                self.a = self.read_byte_hl();
                if self.l.0 == 0x00 {
                    self.h -= 1;
                }
                self.l -= 1;
                self.pc += 1;
                8
            },
            0x3b => { // dec SP
                self.sp -= 1;
                self.sp &= 0xffff;
                self.pc += 1;
                8
            },
            0x3c => { // inc A
                self.a += 1;
                self.f &= 0x10;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_zero(self.a.0 & 0x0f);
                self.pc += 1;
                4
            },
            0x3d => { // dec A
                self.f &= 0x10;
                self.f |= 0x40;
                self.set_h_on_zero(self.a.0 & 0x0f);
                self.a -= 0x01;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x3e => { // ld A, n
                self.a = self.read_address(self.pc + Wrapping(1));
                self.pc += 2;
                8
            },
            0x3f => { // ccf (invert carry flags)
                self.f &= 0xb0;
                let mut temp_byte = self.f.0 & 0x30;
                temp_byte = temp_byte ^ 0x30;
                self.f &= 0x80;
                self.f |= temp_byte;
                self.pc += 1;
                4
            },
            0x40 => { // ld B, B
                self.pc += 1;
                4
            },
            0x41 => { // ld B, C
                self.b = self.c;
                self.pc += 1;
                4
            },
            0x42 => { // ld B, D
                self.b = self.d;
                self.pc += 1;
                4
            },
            0x43 => { // ld B, E
                self.b = self.e;
                self.pc += 1;
                4
            },
            0x44 => { // ld B, H
                self.b = self.h;
                self.pc += 1;
                4
            },
            0x45 => { // ld B, L
                self.b = self.l;
                self.pc += 1;
                4
            },
            0x46 => { // ld B, (HL)
                self.b = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x47 => { // ld B, A
                self.b = self.a;
                self.pc += 1;
                4
            },
            0x48 => { // ld C, B
                self.c = self.b;
                self.pc += 1;
                4
            },
            0x49 => { // ld C, C
                self.pc += 1;
                4
            },
            0x4a => { // ld C, D
                self.c = self.d;
                self.pc += 1;
                4
            },
            0x4b => { // ld C, E
                self.c = self.e;
                self.pc += 1;
                4
            },
            0x4c => { // ld C, H
                self.c = self.h;
                self.pc += 1;
                4
            },
            0x4d => { // ld C, L
                self.c = self.l;
                self.pc += 1;
                4
            },
            0x4e => { // ld C, (HL)
                self.c = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x4f => { // ld C, A
                self.c = self.a;
                self.pc += 1;
                4
            },
            0x50 => { // ld D, B
                self.d = self.b;
                self.pc += 1;
                4
            },
            0x51 => { // ld D, C
                self.d = self.c;
                self.pc += 1;
                4
            },
            0x52 => { // ld D, D
                self.pc += 1;
                4
            },
            0x53 => { // ld D, E
                self.d = self.e;
                self.pc += 1;
                4
            },
            0x54 => { // ld D, H
                self.d = self.h;
                self.pc += 1;
                4
            },
            0x55 => { // ld D, L
                self.d = self.l;
                self.pc += 1;
                4
            },
            0x56 => { // ld D, (HL)
                self.d = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x57 => { // ld D, A
                self.d = self.a;
                self.pc += 1;
                4
            },
            0x58 => { // ld E, B
                self.e = self.b;
                self.pc += 1;
                4
            },
            0x59 => { // ld E, C
                self.e = self.c;
                self.pc += 1;
                4
            },
            0x5a => { // ld E, D
                self.e = self.d;
                self.pc += 1;
                4
            },
            0x5b => { // ld E, E
                self.pc += 1;
                4
            },
            0x5c => { // ld E, H
                self.e = self.h;
                self.pc += 1;
                4
            },
            0x5d => { // ld E, L
                self.e = self.l;
                self.pc += 1;
                4
            },
            0x5e => { // ld E, (HL)
                self.e = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x5f => { // ld E, A
                self.e = self.a;
                self.pc += 1;
                4
            },
            0x60 => { // ld H, B
                self.h = self.b;
                self.pc += 1;
                4
            },
            0x61 => { // ld H, C
                self.h = self.c;
                self.pc += 1;
                4
            },
            0x62 => { // ld H, D
                self.h = self.d;
                self.pc += 1;
                4
            },
            0x63 => { // ld H, E
                self.h = self.e;
                self.pc += 1;
                4
            },
            0x64 => { // ld H, H
                self.pc += 1;
                4
            },
            0x65 => { // ld H, L
                self.h = self.l;
                self.pc += 1;
                4
            },
            0x66 => { // ld H, (HL)
                self.h = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x67 => { // ld H, A
                self.h = self.a;
                self.pc += 1;
                4
            },
            0x68 => { // ld L, B
                self.l = self.b;
                self.pc += 1;
                4
            },
            0x69 => { // ld L, C
                self.l = self.c;
                self.pc += 1;
                4
            },
            0x6a => { // ld L, D
                self.l = self.d;
                self.pc += 1;
                4
            },
            0x6b => { // ld L, E
                self.l = self.e;
                self.pc += 1;
                4
            },
            0x6c => { // ld L, H
                self.l = self.h;
                self.pc += 1;
                4
            },
            0x6d => { // ld L, L
                self.pc += 1;
                4
            },
            0x6e => { // ld L, (HL)
                self.l = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x6f => { // ld L, A
                self.l = self.a;
                self.pc += 1;
                4
            },
            0x70 => { // ld (HL), B
                self.write_byte_hl(self.b);
                self.pc += 1;
                8
            },
            0x71 => { // ld (HL), C
                self.write_byte_hl(self.c);
                self.pc += 1;
                8
            },
            0x72 => { // ld (HL), D
                self.write_byte_hl(self.d);
                self.pc += 1;
                8
            },
            0x73 => { // ld (HL), E
                self.write_byte_hl(self.e);
                self.pc += 1;
                8
            },
            0x74 => { // ld (HL), H
                self.write_byte_hl(self.h);
                self.pc += 1;
                8
            },
            0x75 => { // ld (HL), L
                self.write_byte_hl(self.l);
                self.pc += 1;
                8
            },
            0x76 => { // halt (NOTE THAT THIS GETS INTERRUPTED EVEN WHEN INTERRUPTS ARE DISABLED)
                self.mode = Mode::Halted;
                self.pc += 1;
                4
            },
            0x77 => { // ld (HL), A
                self.write_byte_hl(self.a);
                self.pc += 1;
                8
            },
            0x78 => { // ld A, B
                self.a = self.b;
                self.pc += 1;
                4
            },
            0x79 => { // ld A, C
                self.a = self.c;
                self.pc += 1;
                4
            },
            0x7a => { // ld A, D
                self.a = self.d;
                self.pc += 1;
                4
            },
            0x7b => { // ld A, E
                self.a = self.e;
                self.pc += 1;
                4
            },
            0x7c => { // ld A, H
                self.a = self.h;
                self.pc += 1;
                4
            },
            0x7d => { // ld A, L
                self.a = self.l;
                self.pc += 1;
                4
            },
            0x7e => { // ld A, (HL)
                self.a = self.read_byte_hl();
                self.pc += 1;
                8
            },
            0x7f => { // ld A, A
                self.pc += 1;
                4
            },
            0x80 => { // add B (add B to A)
                self.a += self.b;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.b.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.b > self.a);
                self.pc += 1;
                4
            },
            0x81 => { // add C
                self.a += self.c;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.c.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.c > self.a);
                self.pc += 1;
                4
            },
            0x82 => { // add D
                self.a += self.d;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.d.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.d > self.a);
                self.pc += 1;
                4
            },
            0x83 => { // add E
                self.a += self.e;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.e.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.e > self.a);
                self.pc += 1;
                4
            },
            0x84 => { // add H
                self.a += self.h;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.h.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.h > self.a);
                self.pc += 1;
                4
            },
            0x85 => { // add L
                self.a += self.l;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_h_on_condition((self.l.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.l > self.a);
                self.pc += 1;
                4
            },
            0x86 => { // add (HL)
                let temp_byte = self.read_byte_hl();
                self.a += temp_byte;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(temp_byte.0 > self.a.0);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                8
            },
            0x87 => { // add A
                self.f.0 = 0x00;
                self.set_h_on_condition((self.a.0 & 0x08) != 0x00);
                self.set_c_on_condition((self.a.0 & 0x80) != 0x00);
                self.a += self.a;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x88 => { // adc A, B (add B + carry to A)
                let mut temp_byte = self.b;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x89 => { // adc A, C
                let mut temp_byte = self.c;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x8a => { // adc A, D
                let mut temp_byte = self.d;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x8b => { // adc A, E
                let mut temp_byte = self.e;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x8c => { // adc A, H
                let mut temp_byte = self.h;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x8d => { // adc A, L
                let mut temp_byte = self.l;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x8e => { // adc A, (HL)
                let mut temp_byte = self.read_byte_hl();
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                8
            },
            0x8f => { // adc A, A
                let mut temp_byte = self.a;
                if (self.f.0 & 0x10) != 0x00 {
                    self.f.0 = 0x00;
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                } else {
                    self.f.0 = 0x00;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                4
            },
            0x90 => { // sub B (sub B from A)
                self.f.0 = 0x40;
                self.set_c_on_condition(self.b > self.a);
                self.set_h_on_condition((self.b.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.b;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x91 => { // sub C
                self.f.0 = 0x40;
                self.set_c_on_condition(self.c > self.a);
                self.set_h_on_condition((self.c.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.c;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x92 => { // sub D
                self.f.0 = 0x40;
                self.set_c_on_condition(self.d > self.a);
                self.set_h_on_condition((self.d.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.d;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x93 => { // sub E
                self.f.0 = 0x40;
                self.set_c_on_condition(self.e > self.a);
                self.set_h_on_condition((self.e.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.e;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x94 => { // sub H
                self.f.0 = 0x40;
                self.set_c_on_condition(self.h > self.a);
                self.set_h_on_condition((self.h.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.h;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x95 => { // sub L
                self.f.0 = 0x40;
                self.set_c_on_condition(self.l > self.a);
                self.set_h_on_condition((self.l.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.l;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                4
            },
            0x96 => { // sub (HL)
                let temp_byte = self.read_byte_hl();
                self.f.0 = 0x40;
                self.set_c_on_condition(temp_byte > self.a);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= temp_byte;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 1;
                8
            },
            0x97 => { // sub A
                self.f.0 = 0xc0;
                self.a.0 = 0x00;
                self.pc += 1;
                4
            },
            0x98 => { // sbc A, B (A = A - (B+carry))
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.b > self.a);
                self.set_h_on_condition((self.b.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.b;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x99 => { // sbc A, C
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.c > self.a);
                self.set_h_on_condition((self.c.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.c;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x9a => { // sbc A, D
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.d > self.a);
                self.set_h_on_condition((self.d.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.d;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x9b => { // sbc A, E
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.e > self.a);
                self.set_h_on_condition((self.e.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.e;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x9c => { // sbc A, H
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.h > self.a);
                self.set_h_on_condition((self.h.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.h;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x9d => { // sbc A, L
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(self.l > self.a);
                self.set_h_on_condition((self.l.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= self.l;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0x9e => { // sbc A, (HL)
                let temp_byte = self.read_byte_hl();
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.set_c_on_condition(temp_byte > self.a);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= temp_byte;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                8
            },
            0x9f => { // sbc A, A
                let carry_was_set = (self.f.0 & 0x10) != 0x00;
                self.f.0 = 0x40;
                self.a.0 = 0;
                if carry_was_set {
                    if self.a.0 == 0 {
                        self.a.0 = 0xff;
                        self.f.0 = 0x70;
                    } else {
                        self.a -= 1;
                    }
                }
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa0 => { // and B (and B against A)
                self.a = self.a & self.b;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa1 => { // and C
                self.a = self.a & self.c;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa2 => { // and D
                self.a = self.a & self.d;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa3 => { // and E
                self.a = self.a & self.e;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa4 => { // and H
                self.a = self.a & self.h;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa5 => { // and L
                self.a = self.a & self.l;
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa6 => { // and (HL)
                self.a = self.a & self.read_byte_hl();
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                8
            },
            0xa7 => { // and A
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa8 => { // xor B (A = A XOR B)
                self.a = self.a ^ self.b;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xa9 => { // xor C
                self.a = self.a ^ self.c;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xaa => { // xor D
                self.a = self.a ^ self.d;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xab => { // xor E
                self.a = self.a ^ self.e;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xac => { // xor H
                self.a = self.a ^ self.h;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xad => { // xor L
                self.a = self.a ^ self.l;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xae => { // xor (HL)
                self.a = self.a ^ self.read_byte_hl();
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                8
            },
            0xaf => { // xor A
                self.a.0 = 0x00;
                self.f.0 = 0x80;
                self.pc += 1;
                4
            },
            0xb0 => { // or B (or B against A)
                self.a = self.a | self.b;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb1 => { // or C
                self.a = self.a | self.c;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb2 => { // or D
                self.a = self.a | self.d;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb3 => { // or E
                self.a = self.a | self.e;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb4 => { // or H
                self.a = self.a | self.h;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb5 => { // or L
                self.a = self.a | self.l;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb6 => { // or (HL)
                self.a = self.a | self.read_byte_hl();
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                8
            },
            0xb7 => { // or A
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 1;
                4
            },
            0xb8 => { // cp B
                self.f.0 = 0x40;
                self.set_h_on_condition((self.b.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.b > self.a);
                self.set_z_on_condition(self.a == self.b);
                self.pc += 1;
                4
            },
            0xb9 => { // cp C
                self.f.0 = 0x40;
                self.set_h_on_condition((self.c.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.c > self.a);
                self.set_z_on_condition(self.a == self.c);
                self.pc += 1;
                4
            },
            0xba => { // cp D
                self.f.0 = 0x40;
                self.set_h_on_condition((self.d.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.d > self.a);
                self.set_z_on_condition(self.a == self.d);
                self.pc += 1;
                4
            },
            0xbb => { // cp E
                self.f.0 = 0x40;
                self.set_h_on_condition((self.e.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.e > self.a);
                self.set_z_on_condition(self.a == self.e);
                self.pc += 1;
                4
            },
            0xbc => { // cp H
                self.f.0 = 0x40;
                self.set_h_on_condition((self.h.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.h > self.a);
                self.set_z_on_condition(self.a == self.h);
                self.pc += 1;
                4
            },
            0xbd => { // cp L
                self.f.0 = 0x40;
                self.set_h_on_condition((self.l.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(self.l > self.a);
                self.set_z_on_condition(self.a == self.l);
                self.pc += 1;
                4
            },
            0xbe => { // cp (HL)
                let temp_byte = self.read_byte_hl();
                self.f.0 = 0x40;
                self.set_c_on_condition(temp_byte > self.a);
                self.set_z_on_condition(self.a == temp_byte);
                self.set_h_on_condition((temp_byte.0 & 0x0f) > (self.a.0 & 0x0f));
                self.pc += 1;
                8
            },
            0xbf => { // cp A
                self.f.0 = 0xc0;
                self.pc += 1;
                4
            },
            0xc0 => { // ret NZ
                if (self.f.0 & 0x80) != 0x00 {
                    self.pc += 1;
                    8
                } else {
                    let (msb, lsb) = self.read_address_16(self.sp);
                    self.sp += 2;
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    20
                }
            },
            0xc1 => { // pop BC
                let (c, b) = self.read_address_16(self.sp);
                self.b = b;
                self.c = c;
                self.sp += 2;
                self.pc += 1;
                12
            },
            0xc2 => { // j NZ, nn
                if (self.f.0 & 0x80) != 0x00 {
                    self.pc += 3;
                    12
                } else {
                    self.pc.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                    16
                }
            },
            0xc3 => { // jump to nn
                self.pc.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                16
            },
            0xc4 => { // call NZ, nn
                if (self.f.0 & 0x80) != 0x00 {
                    self.pc += 3;
                    12
                } else {
                    let msb = self.read_address(self.pc + Wrapping(1));
                    let lsb = self.read_address(self.pc + Wrapping(2));
                    self.pc += 3;
                    self.sp -= 2;
                    self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0xff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    24
                }
            },
            0xc5 => { // push BC
                self.sp -= 2;
                self.write_address_16(self.sp, (self.c, self.b));
                self.pc += 1;
                16
            },
            0xc6 => { // add A, n
                let msb = self.read_address(self.pc + Wrapping(1));
                self.a += msb;
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a.0 < msb.0);
                self.set_h_on_condition((self.a.0 & 0x0f) < (msb.0 & 0x0f));
                self.pc += 2;
                8
            },
            0xc7 => { // rst 0 (call routine at 0x0000)
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x00;
                16
            },
            0xc8 => { // ret Z
                if (self.f.0 & 0x80) != 0x00 {
                    let (msb, lsb) = self.read_address_16(self.sp);
                    self.sp += 2;
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    20
                } else {
                    self.pc += 1;
                    8
                }
            },
            0xc9 => { // return
                let (msb, lsb) = self.read_address_16(self.sp);
                self.sp += 2;
                self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                16
            },
            0xca => { // j Z, nn
                if (self.f.0 & 0x80) != 0x00 {
                    self.pc.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                    16
                } else {
                    self.pc += 3;
                    12
                }
            },
            0xcb => { // extended instructions
                self.pc += 2;
                let next_address_byte = self.read_address(self.pc - Wrapping(1)).0;
                match next_address_byte {
                    0x00 => { // rlc B
                        let bit_7_was_set = (self.b.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.b = self.b << 1;
                        if bit_7_was_set {
                            self.b |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x01 => { // rlc C
                        let bit_7_was_set = (self.c.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.c = self.c << 1;
                        if bit_7_was_set {
                            self.c |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x02 => { // rlc D
                        let bit_7_was_set = (self.d.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.d = self.d << 1;
                        if bit_7_was_set {
                            self.d |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x03 => { // rlc E
                        let bit_7_was_set = (self.e.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.e = self.e << 1;
                        if bit_7_was_set {
                            self.e |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x04 => { // rlc H
                        let bit_7_was_set = (self.h.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.h = self.h << 1;
                        if bit_7_was_set {
                            self.h |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x05 => { // rlc L
                        let bit_7_was_set = (self.l.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.l = self.l << 1;
                        if bit_7_was_set {
                            self.l |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x06 => { // rlc (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        let bit_7_was_set = (temp_byte.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        temp_byte = temp_byte << 1;
                        if bit_7_was_set {
                            temp_byte |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x07 => { // rlc A
                        let bit_7_was_set = (self.a.0 & 0x80) != 0;
                        self.f.0 = 0x00; // Reset all other flags
                        self.a = self.a << 1;
                        if bit_7_was_set {
                            self.a |= 0x01;
                            self.f.0 = 0x10; // Set carry
                        }
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x08 => { // rrc B
                        let bit_0_was_set = (self.b.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.b >>= 1;
                        self.b &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.b |= 0x80;
                        }
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x09 => { // rrc C
                        let bit_0_was_set = (self.c.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.c >>= 1;
                        self.c &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.c |= 0x80;
                        }
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x0a => { // rrc D
                        let bit_0_was_set = (self.d.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.d >>= 1;
                        self.d &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.d |= 0x80;
                        }
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x0b => { // rrc E
                        let bit_0_was_set = (self.e.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.e >>= 1;
                        self.e &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.e |= 0x80;
                        }
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x0c => { // rrc H
                        let bit_0_was_set = (self.h.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.h >>= 1;
                        self.h &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.h |= 0x80;
                        }
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x0d => { // rrc L
                        let bit_0_was_set = (self.l.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.l >>= 1;
                        self.l &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.l |= 0x80;
                        }
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x0e => { // rrc (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        let bit_0_was_set = (temp_byte.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        temp_byte >>= 1;
                        temp_byte &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            temp_byte |= 0x80;
                        }
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x0f => { // rrc A
                        let bit_0_was_set = (self.a.0 & 0x01) != 0;
                        self.f.0 = 0x00;
                        self.a >>= 1;
                        self.a &= 0x7f;
                        if bit_0_was_set {
                            self.f.0 = 0x10;
                            self.a |= 0x80;
                        }
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x10 => { // rl B (rotate carry bit to bit 0 of B)
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.b.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.b <<= 1;
                        if carry_was_set {
                            self.b |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x11 => { // rl C
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.c.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.c <<= 1;
                        if carry_was_set {
                            self.c |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x12 => { // rl D
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.d.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.d <<= 1;
                        if carry_was_set {
                            self.d |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x13 => { // rl E
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.e.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.e <<= 1;
                        if carry_was_set {
                            self.e |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x14 => { // rl H
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.h.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.h <<= 1;
                        if carry_was_set {
                            self.h |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x15 => { // rl L
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.l.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.l <<= 1;
                        if carry_was_set {
                            self.l |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x16 => { // rl (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte_2 = self.read_address(temp_addr);
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((temp_byte_2.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        temp_byte_2 <<= 1;
                        if carry_was_set {
                            temp_byte_2 |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(temp_byte_2.0);
                        self.write_address(temp_addr, temp_byte_2);
                        16
                    },
                    0x17 => { // rl A
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.a.0 & 0x80) != 0); // Copy bit 7 to carry bit
                        self.a <<= 1;
                        if carry_was_set {
                            self.a |= 0x01; // Copy carry flag to bit 0
                        }
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x18 => { // rr B (9-bit rotation incl carry bit)
                        let bit_0_was_set = (self.b.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.b >>= 1;
                        self.b &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.b |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x19 => { // rr C
                        let bit_0_was_set = (self.c.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.c >>= 1;
                        self.c &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.c |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x1a => { // rr D
                        let bit_0_was_set = (self.d.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.d >>= 1;
                        self.d &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.d |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x1b => { // rr E
                        let bit_0_was_set = (self.e.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.e >>= 1;
                        self.e &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.e |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x1c => { // rr H
                        let bit_0_was_set = (self.h.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.h >>= 1;
                        self.h &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.h |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x1d => { // rr L
                        let bit_0_was_set = (self.l.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.l >>= 1;
                        self.l &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.l |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x1e => { // rr (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte_3 = self.read_address(temp_addr);
                        let bit_0_was_set = (temp_byte_3.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        temp_byte_3 >>= 1;
                        temp_byte_3 &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            temp_byte_3 |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_condition(temp_byte_3.0 == 0x00);
                        self.write_address(temp_addr, temp_byte_3);
                        16
                    },
                    0x1f => { // rr A
                        let bit_0_was_set = (self.a.0 & 0x01) != 0;
                        let carry_was_set = (self.f.0 & 0x10) != 0x00;
                        self.a >>= 1;
                        self.a &= 0x7f;
                        self.f.0 = 0x00;
                        if carry_was_set {
                            self.a |= 0x80;
                        }
                        self.set_c_on_condition(bit_0_was_set);
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x20 => { // sla B (shift B left arithmetically)
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.b.0 & 0x80) != 0x00);
                        self.b <<= 1;
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x21 => { // sla C
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.c.0 & 0x80) != 0x00);
                        self.c <<= 1;
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x22 => { // sla D
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.d.0 & 0x80) != 0x00);
                        self.d <<= 1;
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x23 => { // sla E
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.e.0 & 0x80) != 0x00);
                        self.e <<= 1;
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x24 => { // sla H
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.h.0 & 0x80) != 0x00);
                        self.h <<= 1;
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x25 => { // sla L
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.l.0 & 0x80) != 0x00);
                        self.l <<= 1;
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x26 => { // sla (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        self.f.0 = 0x00;
                        self.set_c_on_condition((temp_byte.0 & 0x80) != 0x00);
                        temp_byte = temp_byte << 1;
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x27 => { // sla A
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.a.0 & 0x80) != 0x00);
                        self.a <<= 1;
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x28 => { // sra B (shift B right arithmetically - preserve sign bit)
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.b.0 & 0x01) != 0x00);
                        let temp_byte = self.b & Wrapping(0x80);
                        self.b >>= 1;
                        self.b |= temp_byte;
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x29 => { // sra C
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.c.0 & 0x01) != 0x00);
                        let temp_byte = self.c & Wrapping(0x80);
                        self.c >>= 1;
                        self.c |= temp_byte;
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x2a => { // sra D
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.d.0 & 0x01) != 0x00);
                        let temp_byte = self.d & Wrapping(0x80);
                        self.d >>= 1;
                        self.d |= temp_byte;
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x2b => { // sra E
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.e.0 & 0x01) != 0x00);
                        let temp_byte = self.e & Wrapping(0x80);
                        self.e >>= 1;
                        self.e |= temp_byte;
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x2c => { // sra H
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.h.0 & 0x01) != 0x00);
                        let temp_byte = self.h & Wrapping(0x80);
                        self.h >>= 1;
                        self.h |= temp_byte;
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x2d => { // sra L
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.l.0 & 0x01) != 0x00);
                        let temp_byte = self.l & Wrapping(0x80);
                        self.l >>= 1;
                        self.l |= temp_byte;
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x2e => { // sra (HL)
                        self.f.0 = 0x00;
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        self.set_c_on_condition((temp_byte.0 & 0x01) != 0x00);
                        let temp_byte_2 = temp_byte & Wrapping(0x80);
                        temp_byte >>= 1;
                        temp_byte |= temp_byte_2;
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x2f => { // sra A
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.a.0 & 0x01) != 0x00);
                        let temp_byte = self.a & Wrapping(0x80);
                        self.a >>= 1;
                        self.a |= temp_byte;
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x30 => { // swap B
                        let temp_byte = self.b << 4;
                        self.b >>= 4;
                        self.b &= 0x0f;
                        self.b |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x31 => { // swap C
                        let temp_byte = self.c << 4;
                        self.c >>= 4;
                        self.c &= 0x0f;
                        self.c |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x32 => { // swap D
                        let temp_byte = self.d << 4;
                        self.d >>= 4;
                        self.d &= 0x0f;
                        self.d |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x33 => { // swap E
                        let temp_byte = self.e << 4;
                        self.e >>= 4;
                        self.e &= 0x0f;
                        self.e |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x34 => { // swap H
                        let temp_byte = self.h << 4;
                        self.h >>= 4;
                        self.h &= 0x0f;
                        self.h |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x35 => { // swap L
                        let temp_byte = self.l << 4;
                        self.l >>= 4;
                        self.l &= 0x0f;
                        self.l |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x36 => { // swap (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        let temp_byte_2 = temp_byte << 4;
                        temp_byte >>= 4;
                        temp_byte &= 0x0f;
                        temp_byte |= temp_byte_2;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x37 => { // swap A
                        let temp_byte = self.a << 4;
                        self.a >>= 4;
                        self.a &= 0x0f;
                        self.a |= temp_byte;
                        self.f.0 = 0x00;
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x38 => { // srl B
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.b.0 & 0x01) != 0x00);
                        self.b >>= 1;
                        self.b &= 0x7f;
                        self.set_z_on_zero(self.b.0);
                        8
                    },
                    0x39 => { // srl C
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.c.0 & 0x01) != 0x00);
                        self.c >>= 1;
                        self.c &= 0x7f;
                        self.set_z_on_zero(self.c.0);
                        8
                    },
                    0x3a => { // srl D
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.d.0 & 0x01) != 0x00);
                        self.d >>= 1;
                        self.d &= 0x7f;
                        self.set_z_on_zero(self.d.0);
                        8
                    },
                    0x3b => { // srl E
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.e.0 & 0x01) != 0x00);
                        self.e >>= 1;
                        self.e &= 0x7f;
                        self.set_z_on_zero(self.e.0);
                        8
                    },
                    0x3c => { // srl H
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.h.0 & 0x01) != 0x00);
                        self.h >>= 1;
                        self.h &= 0x7f;
                        self.set_z_on_zero(self.h.0);
                        8
                    },
                    0x3d => { // srl L
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.l.0 & 0x01) != 0x00);
                        self.l >>= 1;
                        self.l &= 0x7f;
                        self.set_z_on_zero(self.l.0);
                        8
                    },
                    0x3e => { // srl (HL)
                        let temp_addr = self.get_address_hl();
                        let mut temp_byte = self.read_address(temp_addr);
                        self.f.0 = 0x00;
                        self.set_c_on_condition((temp_byte.0 & 0x01) != 0x00);
                        temp_byte >>= 1;
                        temp_byte &= 0x7f;
                        self.set_z_on_zero(temp_byte.0);
                        self.write_address(temp_addr, temp_byte);
                        16
                    },
                    0x3f => { // srl A
                        self.f.0 = 0x00;
                        self.set_c_on_condition((self.a.0 & 0x01) != 0x00);
                        self.a >>= 1;
                        self.a &= 0x7f;
                        self.set_z_on_zero(self.a.0);
                        8
                    },
                    0x40 => { // Test bit 0 of B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x01);
                        8
                    },
                    0x41 => { // Test bit 0 of C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x01);
                        8
                    },
                    0x42 => { // Test bit 0 of D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x01);
                        8
                    },
                    0x43 => { // Test bit 0 of E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x01);
                        8
                    },
                    0x44 => { // Test bit 0 of H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x01);
                        8
                    },
                    0x45 => { // Test bit 0 of L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x01);
                        8
                    },
                    0x46 => { // Test bit 0 of (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x01);
                        12
                    },
                    0x47 => { // Test bit 0 of A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x01);
                        8
                    },
                    0x48 => { // bit 1, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x02);
                        8
                    },
                    0x49 => { // bit 1, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x02);
                        8
                    },
                    0x4a => { // bit 1, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x02);
                        8
                    },
                    0x4b => { // bit 1, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x02);
                        8
                    },
                    0x4c => { // bit 1, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x02);
                        8
                    },
                    0x4d => { // bit 1, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x02);
                        8
                    },
                    0x4e => { // bit 1, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x02);
                        12
                    },
                    0x4f => { // bit 1, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x02);
                        8
                    },
                    0x50 => { // bit 2, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x04);
                        8
                    },
                    0x51 => { // bit 2, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x04);
                        8
                    },
                    0x52 => { // bit 2, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x04);
                        8
                    },
                    0x53 => { // bit 2, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x04);
                        8
                    },
                    0x54 => { // bit 2, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x04);
                        8
                    },
                    0x55 => { // bit 2, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x04);
                        8
                    },
                    0x56 => { // bit 2, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x04);
                        12
                    },
                    0x57 => { // bit 2, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x04);
                        8
                    },
                    0x58 => { // bit 3, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x08);
                        8
                    },
                    0x59 => { // bit 3, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x08);
                        8
                    },
                    0x5a => { // bit 3, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x08);
                        8
                    },
                    0x5b => { // bit 3, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x08);
                        8
                    },
                    0x5c => { // bit 3, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x08);
                        8
                    },
                    0x5d => { // bit 3, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x08);
                        8
                    },
                    0x5e => { // bit 3, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x08);
                        12
                    },
                    0x5f => { // bit 3, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x08);
                        8
                    },
                    0x60 => { // bit 4, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x10);
                        8
                    },
                    0x61 => { // bit 4, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x10);
                        8
                    },
                    0x62 => { // bit 4, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x10);
                        8
                    },
                    0x63 => { // bit 4, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x10);
                        8
                    },
                    0x64 => { // bit 4, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x10);
                        8
                    },
                    0x65 => { // bit 4, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x10);
                        8
                    },
                    0x66 => { // bit 4, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x10);
                        12
                    },
                    0x67 => { // bit 4, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x10);
                        8
                    },
                    0x68 => { // bit 5, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x20);
                        8
                    },
                    0x69 => { // bit 5, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x20);
                        8
                    },
                    0x6a => { // bit 5, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x20);
                        8
                    },
                    0x6b => { // bit 5, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x20);
                        8
                    },
                    0x6c => { // bit 5, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x20);
                        8
                    },
                    0x6d => { // bit 5, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x20);
                        8
                    },
                    0x6e => { // bit 5, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x20);
                        12
                    },
                    0x6f => { // bit 5, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x20);
                        8
                    },
                    0x70 => { // bit 6, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x40);
                        8
                    },
                    0x71 => { // bit 6, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x40);
                        8
                    },
                    0x72 => { // bit 6, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x40);
                        8
                    },
                    0x73 => { // bit 6, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x40);
                        8
                    },
                    0x74 => { // bit 6, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x40);
                        8
                    },
                    0x75 => { // bit 6, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x40);
                        8
                    },
                    0x76 => { // bit 6, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x40);
                        12
                    },
                    0x77 => { // bit 6, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x40);
                        8
                    },
                    0x78 => { // bit 7, B
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.b.0 & 0x80);
                        8
                    },
                    0x79 => { // bit 7, C
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.c.0 & 0x80);
                        8
                    },
                    0x7a => { // bit 7, D
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.d.0 & 0x80);
                        8
                    },
                    0x7b => { // bit 7, E
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.e.0 & 0x80);
                        8
                    },
                    0x7c => { // bit 7, H
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.h.0 & 0x80);
                        8
                    },
                    0x7d => { // bit 7, L
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.l.0 & 0x80);
                        8
                    },
                    0x7e => { // bit 7, (HL)
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.read_byte_hl().0 & 0x80);
                        12
                    },
                    0x7f => { // bit 7, A
                        self.f &= 0x30;
                        self.f |= 0x20;
                        self.set_z_on_zero(self.a.0 & 0x80);
                        8
                    },
                    0x80 => { // res 0, B
                        self.b &= 0xfe;
                        8
                    },
                    0x81 => { // res 0, C
                        self.c &= 0xfe;
                        8
                    },
                    0x82 => { // res 0, D
                        self.d &= 0xfe;
                        8
                    },
                    0x83 => { // res 0, E
                        self.e &= 0xfe;
                        8
                    },
                    0x84 => { // res 0, H
                        self.h &= 0xfe;
                        8
                    },
                    0x85 => { // res 0, L
                        self.l &= 0xfe;
                        8
                    },
                    0x86 => { // res 0, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xfe));
                        16
                    },
                    0x87 => { // res 0, A
                        self.a &= 0xfe;
                        8
                    },
                    0x88 => { // res 1, B
                        self.b &= 0xfd;
                        8
                    },
                    0x89 => { // res 1, C
                        self.c &= 0xfd;
                        8
                    },
                    0x8a => { // res 1, D
                        self.d &= 0xfd;
                        8
                    },
                    0x8b => { // res 1, E
                        self.e &= 0xfd;
                        8
                    },
                    0x8c => { // res 1, H
                        self.h &= 0xfd;
                        8
                    },
                    0x8d => { // res 1, L
                        self.l &= 0xfd;
                        8
                    },
                    0x8e => { // res 1, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xfd));
                        16
                    },
                    0x8f => { // res 1, A
                        self.a &= 0xfd;
                        8
                    },
                    0x90 => { // res 2, B
                        self.b &= 0xfb;
                        8
                    },
                    0x91 => { // res 2, C
                        self.c &= 0xfb;
                        8
                    },
                    0x92 => { // res 2, D
                        self.d &= 0xfb;
                        8
                    },
                    0x93 => { // res 2, E
                        self.e &= 0xfb;
                        8
                    },
                    0x94 => { // res 2, H
                        self.h &= 0xfb;
                        8
                    },
                    0x95 => { // res 2, L
                        self.l &= 0xfb;
                        8
                    },
                    0x96 => { // res 2, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xfb));
                        16
                    },
                    0x97 => { // res 2, A
                        self.a &= 0xfb;
                        8
                    },
                    0x98 => { // res 3, B
                        self.b &= 0xf7;
                        8
                    },
                    0x99 => { // res 3, C
                        self.c &= 0xf7;
                        8
                    },
                    0x9a => { // res 3, D
                        self.d &= 0xf7;
                        8
                    },
                    0x9b => { // res 3, E
                        self.e &= 0xf7;
                        8
                    },
                    0x9c => { // res 3, H
                        self.h &= 0xf7;
                        8
                    },
                    0x9d => { // res 3, L
                        self.l &= 0xf7;
                        8
                    },
                    0x9e => { // res 3, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xf7));
                        16
                    },
                    0x9f => { // res 3, A
                        self.a &= 0xf7;
                        8
                    },
                    0xa0 => { // res 4, B
                        self.b &= 0xef;
                        8
                    },
                    0xa1 => { // res 4, C
                        self.c &= 0xef;
                        8
                    },
                    0xa2 => { // res 4, D
                        self.d &= 0xef;
                        8
                    },
                    0xa3 => { // res 4, E
                        self.e &= 0xef;
                        8
                    },
                    0xa4 => { // res 4, H
                        self.h &= 0xef;
                        8
                    },
                    0xa5 => { // res 4, L
                        self.l &= 0xef;
                        8
                    },
                    0xa6 => { // res 4, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xef));
                        16
                    },
                    0xa7 => { // res 4, A
                        self.a &= 0xef;
                        8
                    },
                    0xa8 => { // res 5, B
                        self.b &= 0xdf;
                        8
                    },
                    0xa9 => { // res 5, C
                        self.c &= 0xdf;
                        8
                    },
                    0xaa => { // res 5, D
                        self.d &= 0xdf;
                        8
                    },
                    0xab => { // res 5, E
                        self.e &= 0xdf;
                        8
                    },
                    0xac => { // res 5, H
                        self.h &= 0xdf;
                        8
                    },
                    0xad => { // res 5, L
                        self.l &= 0xdf;
                        8
                    },
                    0xae => { // res 5, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xdf));
                        16
                    },
                    0xaf => { // res 5, A
                        self.a &= 0xdf;
                        8
                    },
                    0xb0 => { // res 6, B
                        self.b &= 0xbf;
                        8
                    },
                    0xb1 => { // res 6, C
                        self.c &= 0xbf;
                        8
                    },
                    0xb2 => { // res 6, D
                        self.d &= 0xbf;
                        8
                    },
                    0xb3 => { // res 6, E
                        self.e &= 0xbf;
                        8
                    },
                    0xb4 => { // res 6, H
                        self.h &= 0xbf;
                        8
                    },
                    0xb5 => { // res 6, L
                        self.l &= 0xbf;
                        8
                    },
                    0xb6 => { // res 6, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0xbf));
                        16
                    },
                    0xb7 => { // res 6, A
                        self.a &= 0xbf;
                        8
                    },
                    0xb8 => { // res 7, B
                        self.b &= 0x7f;
                        8
                    },
                    0xb9 => { // res 7, C
                        self.c &= 0x7f;
                        8
                    },
                    0xba => { // res 7, D
                        self.d &= 0x7f;
                        8
                    },
                    0xbb => { // res 7, E
                        self.e &= 0x7f;
                        8
                    },
                    0xbc => { // res 7, H
                        self.h &= 0x7f;
                        8
                    },
                    0xbd => { // res 7, L
                        self.l &= 0x7f;
                        8
                    },
                    0xbe => { // res 7, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) & Wrapping(0x7f));
                        16
                    },
                    0xbf => { // res 7, A
                        self.a &= 0x7f;
                        8
                    },
                    0xc0 => { // set 0, B
                        self.b |= 0x01;
                        8
                    },
                    0xc1 => { // set 0, C
                        self.c |= 0x01;
                        8
                    },
                    0xc2 => { // set 0, D
                        self.d |= 0x01;
                        8
                    },
                    0xc3 => { // set 0, E
                        self.e |= 0x01;
                        8
                    },
                    0xc4 => { // set 0, H
                        self.h |= 0x01;
                        8
                    },
                    0xc5 => { // set 0, L
                        self.l |= 0x01;
                        8
                    },
                    0xc6 => { // set 0, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x01));
                        16
                    },
                    0xc7 => { // set 0, A
                        self.a |= 0x01;
                        8
                    },
                    0xc8 => { // set 1, B
                        self.b |= 0x02;
                        8
                    },
                    0xc9 => { // set 1, C
                        self.c |= 0x02;
                        8
                    },
                    0xca => { // set 1, D
                        self.d |= 0x02;
                        8
                    },
                    0xcb => { // set 1, E
                        self.e |= 0x02;
                        8
                    },
                    0xcc => { // set 1, H
                        self.h |= 0x02;
                        8
                    },
                    0xcd => { // set 1, L
                        self.l |= 0x02;
                        8
                    },
                    0xce => { // set 1, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x02));
                        16
                    },
                    0xcf => { // set 1, A
                        self.a |= 0x02;
                        8
                    },
                    0xd0 => { // set 2, B
                        self.b |= 0x04;
                        8
                    },
                    0xd1 => { // set 2, C
                        self.c |= 0x04;
                        8
                    },
                    0xd2 => { // set 2, D
                        self.d |= 0x04;
                        8
                    },
                    0xd3 => { // set 2, E
                        self.e |= 0x04;
                        8
                    },
                    0xd4 => { // set 2, H
                        self.h |= 0x04;
                        8
                    },
                    0xd5 => { // set 2, L
                        self.l |= 0x04;
                        8
                    },
                    0xd6 => { // set 2, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x04));
                        16
                    },
                    0xd7 => { // set 2, A
                        self.a |= 0x04;
                        8
                    },
                    0xd8 => { // set 3, B
                        self.b |= 0x08;
                        8
                    },
                    0xd9 => { // set 3, C
                        self.c |= 0x08;
                        8
                    },
                    0xda => { // set 3, D
                        self.d |= 0x08;
                        8
                    },
                    0xdb => { // set 3, E
                        self.e |= 0x08;
                        8
                    },
                    0xdc => { // set 3, H
                        self.h |= 0x08;
                        8
                    },
                    0xdd => { // set 3, L
                        self.l |= 0x08;
                        8
                    },
                    0xde => { // set 3, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x08));
                        16
                    },
                    0xdf => { // set 3, A
                        self.a |= 0x08;
                        8
                    },
                    0xe0 => { // set 4, B
                        self.b |= 0x10;
                        8
                    },
                    0xe1 => { // set 4, C
                        self.c |= 0x10;
                        8
                    },
                    0xe2 => { // set 4, D
                        self.d |= 0x10;
                        8
                    },
                    0xe3 => { // set 4, E
                        self.e |= 0x10;
                        8
                    },
                    0xe4 => { // set 4, H
                        self.h |= 0x10;
                        8
                    },
                    0xe5 => { // set 4, L
                        self.l |= 0x10;
                        8
                    },
                    0xe6 => { // set 4, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x10));
                        16
                    },
                    0xe7 => { // set 4, A
                        self.a |= 0x10;
                        8
                    },
                    0xe8 => { // set 5, B
                        self.b |= 0x20;
                        8
                    },
                    0xe9 => { // set 5, C
                        self.c |= 0x20;
                        8
                    },
                    0xea => { // set 5, D
                        self.d |= 0x20;
                        8
                    },
                    0xeb => { // set 5, E
                        self.e |= 0x20;
                        8
                    },
                    0xec => { // set 5, H
                        self.h |= 0x20;
                        8
                    },
                    0xed => { // set 5, L
                        self.l |= 0x20;
                        8
                    },
                    0xee => { // set 5, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x20));
                        16
                    },
                    0xef => { // set 5, A
                        self.a |= 0x20;
                        8
                    },
                    0xf0 => { // set 6, B
                        self.b |= 0x40;
                        8
                    },
                    0xf1 => { // set 6, C
                        self.c |= 0x40;
                        8
                    },
                    0xf2 => { // set 6, D
                        self.d |= 0x40;
                        8
                    },
                    0xf3 => { // set 6, E
                        self.e |= 0x40;
                        8
                    },
                    0xf4 => { // set 6, H
                        self.h |= 0x40;
                        8
                    },
                    0xf5 => { // set 6, L
                        self.l |= 0x40;
                        8
                    },
                    0xf6 => { // set 6, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x40));
                        16
                    },
                    0xf7 => { // set 6, A
                        self.a |= 0x40;
                        8
                    },
                    0xf8 => { // set 7, B
                        self.b |= 0x80;
                        8
                    },
                    0xf9 => { // set 7, C
                        self.c |= 0x80;
                        8
                    },
                    0xfa => { // set 7, D
                        self.d |= 0x80;
                        8
                    },
                    0xfb => { // set 7, E
                        self.e |= 0x80;
                        8
                    },
                    0xfc => { // set 7, H
                        self.h |= 0x80;
                        8
                    },
                    0xfd => { // set 7, L
                        self.l |= 0x80;
                        8
                    },
                    0xfe => { // set 7, (HL)
                        let temp_addr = self.get_address_hl();
                        self.write_address(temp_addr, self.read_address(temp_addr) | Wrapping(0x80));
                        16
                    },
                    0xff => { // set 7, A
                        self.a |= 0x80;
                        8
                    }
                }
            },
            0xcc => { // call Z, nn
                if (self.f.0 & 0x80) != 0x00 {
                    let msb = self.read_address(self.pc + Wrapping(1));
                    let lsb = self.read_address(self.pc + Wrapping(2));
                    self.sp -= 2;
                    self.pc += 3;
                    self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    24
                } else {
                    self.pc += 3;
                    12
                }
            },
            0xcd => { // call nn
                let msb = self.read_address(self.pc + Wrapping(1));
                let lsb = self.read_address(self.pc + Wrapping(2));
                self.sp -= 2;
                self.pc += 3;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                24
            },
            0xce => { // adc A, n
                let mut temp_byte = self.read_address(self.pc + Wrapping(1));
                let mut temp_byte_2 = self.f & Wrapping(0x10);
                self.f.0 = 0x00;
                if temp_byte_2.0 != 0x00 {
                    self.set_c_on_condition(temp_byte.0 == 0xff);
                    temp_byte += 1;
                }
                self.a += temp_byte;
                self.set_z_on_zero(self.a.0);
                self.set_c_on_condition(self.a < temp_byte);
                temp_byte &= 0x0f;
                temp_byte_2 = self.a & Wrapping(0x0f);
                self.set_h_on_condition(temp_byte > temp_byte_2);
                self.pc += 2;
                8
            },
            0xcf => { // rst 8 (call 0x0008)
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0xff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0008;
                16
            },
            0xd0 => { // ret NC
                if (self.f.0 & 0x10) != 0x00 {
                    self.pc += 1;
                    8
                } else {
                    let (msb, lsb) = self.read_address_16(self.sp);
                    self.sp += 2;
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    20
                }
            },
            0xd1 => { // pop DE
                let (e, d) = self.read_address_16(self.sp);
                self.d = d;
                self.e = e;
                self.sp += 2;
                self.pc += 1;
                12
            },
            0xd2 => { // j NC, nn
                if (self.f.0 & 0x10) != 0x00 {
                    self.pc += 3;
                    12
                } else {
                    self.pc.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                    16
                }
            },
            0xd3 => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xd4 => { // call NC, nn
                if (self.f.0 & 0x10) != 0x00 {
                    self.pc += 3;
                    12
                } else {
                    let msb = self.read_address(self.pc + Wrapping(1));
                    let lsb = self.read_address(self.pc + Wrapping(2));
                    self.sp -= 2;
                    self.pc += 3;
                    self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    24
                }
            },
            0xd5 => { // push DE
                self.sp -= 2;
                self.write_address_16(self.sp, (self.e, self.d));
                self.pc += 1;
                16
            },
            0xd6 => { // sub A, n
                let msb = self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x40;
                self.set_c_on_condition(msb > self.a);
                self.set_h_on_condition((msb.0 & 0x0f) > (self.a.0 & 0x0f));
                self.a -= msb;
                if self.a.0 == 0x00 {
                    self.f.0 = 0xc0;
                }
                self.pc += 2;
                8
            },
            0xd7 => { // rst 10
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0010;
                16
            },
            0xd8 => { // ret C
                if (self.f.0 & 0x10) != 0x00 {
                    let (msb, lsb) = self.read_address_16(self.sp);
                    self.sp += 2;
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    20
                } else {
                    self.pc += 1;
                    8
                }
            },
            0xd9 => { // reti
                let (msb, lsb) = self.read_address_16(self.sp);
                self.sp += 2;
                self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                self.ime = true;
                16
            },
            0xda => { // j C, nn (abs jump if carry)
                if (self.f.0 & 0x10) != 0x00 {
                    self.pc.0 = ((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize;
                    16
                } else {
                    self.pc += 3;
                    12
                }
            },
            0xdb => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xdc => { // call C, nn
                if (self.f.0 & 0x10) != 0x00 {
                    let msb = self.read_address(self.pc + Wrapping(1));
                    let lsb = self.read_address(self.pc + Wrapping(2));
                    self.sp -= 2;
                    self.pc += 3;
                    self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                    self.pc.0 = ((lsb.0 as usize) << 8) + msb.0 as usize;
                    24
                } else {
                    self.pc += 3;
                    12
                }
            },
            0xdd => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xde => { // sbc A, n
                let mut temp_byte = self.a;
                let mut temp_byte_2 = self.f & Wrapping(0x10);
                self.f.0 = 0x40;
                self.a -= self.read_address(self.pc + Wrapping(1));
                if temp_byte_2.0 != 0x00 {
                    if self.a.0 == 0x00 {
                        self.f |= 0x30;
                    }
                    self.a -= 1;
                }
                self.set_c_on_condition(self.a > temp_byte);
                self.set_z_on_zero(self.a.0);
                temp_byte_2 &= 0x0f;
                temp_byte = self.a & Wrapping(0x0f);
                self.set_h_on_condition(temp_byte > temp_byte_2);
                self.pc += 2;
                8
            },
            0xdf => { // rst 18
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0018;
                16
            },
            0xe0 => { // ldh (n), A (load to IO port n - ff00 + n)
                self.write_address(Wrapping(0xff00 + self.read_address(self.pc + Wrapping(1)).0 as usize), self.a);
                self.pc += 2;
                12
            },
            0xe1 => { // pop HL
                let (l, h) = self.read_address_16(self.sp);
                self.h = h;
                self.l = l;
                self.sp += 2;
                self.pc += 1;
                12
            },
            0xe2 => { // ldh (C), A (load to IO port C - ff00 + C)
                self.write_address(Wrapping(0xff00 + self.c.0 as usize), self.a);
                self.pc += 1;
                8
            },
            0xe3 => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xe4 => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xe5 => { // push HL
                self.sp -= 2;
                self.write_address_16(self.sp, (self.l, self.h));
                self.pc += 1;
                16
            },
            0xe6 => { // and n
                self.a &= self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x20;
                self.set_z_on_zero(self.a.0);
                self.pc += 2;
                8
            },
            0xe7 => { // rst 20
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0020;
                16
            },
            0xe8 => { // add SP, d
                let msb = self.read_address(self.pc + Wrapping(1)).0;
                self.f.0 = 0x00;
                if msb >= 0x80 {
                    let temp_addr = 256 - msb as usize;
                    self.sp -= temp_addr;
                    self.set_c_on_condition((self.sp.0 & 0x0000ffff) > (temp_addr & 0x0000ffff));
                    self.set_h_on_condition((self.sp.0 & 0x000000ff) > (temp_addr & 0x000000ff));
                } else {
                    let temp_addr = msb as usize;
                    self.sp += temp_addr;
                    self.set_c_on_condition((self.sp.0 & 0x0000ffff) < (temp_addr & 0x0000ffff));
                    self.set_h_on_condition((self.sp.0 & 0x000000ff) < (temp_addr & 0x000000ff));
                }
                self.pc += 2;
                16
            },
            0xe9 => { // j HL
                self.pc = self.get_address_hl();
                4
            },
            0xea => { // ld (nn), A
                self.write_address(Wrapping(((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize), self.a);
                self.pc += 3;
                16
            },
            0xeb => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xec => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xed => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xee => { // xor n
                self.a ^= self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 2;
                8
            },
            0xef => { // rst 28
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0028;
                16
            },
            0xf0 => { // ldh A, (n)
                self.a = self.read_address(Wrapping(0xff00 + self.read_address(self.pc + Wrapping(1)).0 as usize));
                self.pc += 2;
                12
            },
            0xf1 => { // pop AF
                let (f, a) = self.read_address_16(self.sp);
                self.a = a;
                self.f = f;
                self.f &= 0xf0;
                self.sp += 2;
                self.pc += 1;
                12
            },
            0xf2 => { // ldh A, C
                self.a = self.read_address(Wrapping(0xff00 + self.c.0 as usize));
                self.pc += 1;
                8
            },
            0xf3 => { // di
                self.ime = false;
                self.pc += 1;
                4
            },
            0xf4 => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xf5 => { // push AF
                self.sp -= 2;
                self.write_address_16(self.sp, (self.f, self.a));
                self.pc += 1;
                16
            },
            0xf6 => { // or n
                self.a = self.a | self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x00;
                self.set_z_on_zero(self.a.0);
                self.pc += 2;
                8
            },
            0xf7 => { // rst 30
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0030;
                16
            },
            0xf8 => { // ld HL, SP+d
                let msb = self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x00;
                let mut temp_addr = self.sp;
                if msb.0 >= 0x80 {
                    temp_addr -= 256 - msb.0 as usize;
                    self.set_c_on_condition(temp_addr > self.sp);
                    self.set_h_on_condition((temp_addr.0 & 0x00ffffff) > (self.sp.0 & 0x00ffffff));
                } else {
                    temp_addr += msb.0 as usize;
                    self.set_c_on_condition(self.sp > temp_addr);
                    self.set_h_on_condition((self.sp.0 & 0x00ffffff) > (temp_addr.0 & 0x00ffffff));
                }
                self.h.0 = (temp_addr.0 >> 8) as u8;
                self.l.0 = (temp_addr.0 & 0x00ff) as u8;
                self.pc += 2;
                12
            },
            0xf9 => { // ld SP, HL
                self.sp = self.get_address_hl();
                self.pc += 1;
                8
            },
            0xfa => { // ld A, (nn)
                self.a = self.read_address(Wrapping(((self.read_address(self.pc + Wrapping(2)).0 as usize) << 8) + self.read_address(self.pc + Wrapping(1)).0 as usize));
                self.pc += 3;
                16
            },
            0xfb => { // ei
                self.pc += 1;
                self.ime = true;
                4
            },
            0xfc => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xfd => { // REMOVED INSTRUCTION
                self.on_invalid_instruction(instr)
            },
            0xfe => { // cp n
                let msb = self.read_address(self.pc + Wrapping(1));
                self.f.0 = 0x40;
                self.set_h_on_condition((msb.0 & 0x0f) > (self.a.0 & 0x0f));
                self.set_c_on_condition(msb > self.a);
                self.set_z_on_condition(self.a == msb);
                self.pc += 2;
                8
            },
            0xff => { // rst 38
                self.sp -= 2;
                self.pc += 1;
                self.write_address_16(self.sp, (Wrapping((self.pc.0 & 0x00ff) as u8), Wrapping((self.pc.0 >> 8) as u8)));
                self.pc.0 = 0x0038;
                16
            },
            _ => 0
        }
    }

    pub fn read_address(&self, address: Wrapping<usize>) -> Wrapping<u8> {
        let masked_address = address & Wrapping(0xffff);
        match masked_address.0 {
            0x0000..=0x3fff => self.rom.data[masked_address.0],
            0x4000..=0x7fff => self.rom.data[(self.rom_bank_offset + masked_address).0],
            0x8000..=0x9fff => self.vram[(self.vram_bank_offset + masked_address - Wrapping(0x8000)).0],
            0xa000..=0xbfff => {
                if let Some(sram) = &self.sram {
                    if sram.has_timer && sram.timer_mode > 0 {
                        // TODO - Check this bit
                        sram.timer_data[sram.timer_mode as usize - 0x08]
                    } else {
                        sram.read_byte(masked_address - Wrapping(0xa000))
                    }
                } else {
                    Wrapping(0xff)
                }
            },
            0xc000..=0xcfff => self.wram[(masked_address - Wrapping(0xc000)).0],
            0xd000..=0xdfff => self.wram[(self.wram_bank_offset + masked_address - Wrapping(0xd000)).0],
            0xe000..=0xefff => self.wram[(masked_address - Wrapping(0xe000)).0],
            0xf000..=0xfdff => self.wram[(self.wram_bank_offset + masked_address - Wrapping(0xf000)).0],
            0xfe00..=0xfe9f => {
                if self.oam_protected {
                    Wrapping(0xff)
                } else {
                    self.oam[(masked_address & Wrapping(0x00ff)).0 % 160]
                }
            },
            0xfea0..=0xfeff => Wrapping(0xff),
            0xff00..=0xff7f => self.read_address(masked_address & Wrapping(0x007f)),
            _ => self.io_ports[(masked_address & Wrapping(0x00ff)).0]
        }
    }

    pub fn write_address(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>) {
        let masked_address = address & Wrapping(0xffff);
        match masked_address.0 {
            0x0000..=0x7fff => {
                match self.rom.mbc {
                    Mbc::None => {},
                    Mbc::Mbc1 => {
                        match masked_address.0 {
                            0x0000..=0x1fff => {
                                // Only 4 bits are used. Writing 0xa enables SRAM, anything else disables it
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & Wrapping(0x0f)).0 == 0x0a;
                                }
                            },
                            0x2000..=0x3fff => {
                                // Set low 5 bits of bank number
                                self.rom_bank_offset &= 0xfff80000;
                                let mut byte = byte & Wrapping(0x1f);
                                if byte.0 == 0x00 {
                                    byte = Wrapping(0x01);
                                }
                                self.rom_bank_offset |= byte.0 as usize * 0x4000;
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                let byte = byte & Wrapping(0x03);
                                if let Some(sram) = &mut self.sram {
                                    if self.rom_mbc_mode != 0 {
                                        sram.bank_offset = Wrapping(byte.0 as usize) * Wrapping(0x2000); // Select RAM bank
                                    } else {
                                        self.rom_bank_offset &= 0xffe7c000;
                                        self.rom_bank_offset |= byte.0 as usize * 0x80000;
                                        self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                                    }
                                } else {
                                    // TODO - Check for anything needed here
                                }
                            },
                            _ => {
                                self.rom_mbc_mode = 0;
                                if let Some(sram) = &self.sram {
                                    if sram.size_bytes > 8192 {
                                        self.rom_mbc_mode = byte.0 as u32 & 0x01;
                                    }
                                }
                            }
                        }
                    },
                    Mbc::Mbc2 => {
                        match masked_address.0 {
                            0x0000..=0x0fff => {
                                if let Some(sram) = &mut self.sram {
                                    // Only 4 bits are used. Writing 0xa enables SRAM.
                                    sram.enable_flag = (byte.0 & 0x0f) == 0x0a;
                                }
                            },
                            0x2100..=0x21fe => {
                                let mut byte = (byte & Wrapping(0x0f)) & Wrapping(self.rom.bank_select_mask as u8);
                                if byte.0 == 0x00 {
                                    byte = Wrapping(0x01);
                                }
                                self.rom_bank_offset = Wrapping(byte.0 as usize * 0x4000);
                            },
                            _ => {}
                        }
                    },
                    Mbc::Mbc3 => {
                        match masked_address.0 {
                            0x0000..=0x1fff => {
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte.0 & 0x0f) == 0x0a; // Also enables timer registers
                                }
                            },
                            0x2000..=0x3fff => {
                                let mut byte = byte & Wrapping(self.rom.bank_select_mask as u8);
                                if byte.0 == 0x00 {
                                    byte = Wrapping(0x01);
                                }
                                self.rom_bank_offset = Wrapping(byte.0 as usize * 0x4000);
                            },
                            0x4000..=0x5fff => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & Wrapping(0x0f);
                                    if byte.0 < 0x04 {
                                        sram.bank_offset = Wrapping(byte.0 as usize) * Wrapping(0x2000);
                                        sram.timer_mode = 0;
                                    } else if (byte.0 >= 0x08) && (byte.0 < 0x0d) {
                                        sram.timer_mode = byte.0 as u32;
                                    } else {
                                        sram.timer_mode = 0;
                                    }
                                }
                            },
                            _ => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & Wrapping(0x01);
                                    if (sram.timer_latch == 0x00) && (byte.0 == 0x01) {
                                        sram.latch_timer_data();
                                    }
                                    sram.timer_latch = byte.0 as u32;
                                }
                            }
                        }
                    },
                    Mbc::Mbc5 => {
                        match masked_address.0 {
                            0x0000..=0x1fff => {
                                // RAMG - 4 bits, enable external RAM by writing 0xa
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte.0 & 0x0f) == 0x0a;
                                }
                            },
                            0x2000..=0x2fff => {
                                // ROMB0 - lower 8 bits of 9-bit ROM bank (note MBC5 can select bank 0 here)
                                let masked_byte = byte.0 as usize & self.rom.bank_select_mask as usize;
                                self.rom_bank_offset = (self.rom_bank_offset & Wrapping(0x00400000)) | Wrapping(masked_byte * 0x4000);
                            },
                            0x3000..=0x3fff => {
                                // ROMB1 - 1 bit, upper bit of 9-bit RAM bank (note MBC5 can select bank 0 here)
                                self.rom_bank_offset &= 0x003fc000;
                                if (byte.0 & 0x01) != 0x00 {
                                    self.rom_bank_offset |= 0x00004000;
                                }
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                // RAMB - 4-bit RAM bank
                                if let Some(sram) = &mut self.sram {
                                    sram.bank_offset = Wrapping((byte.0 & 0x0f) as usize) * Wrapping(0x2000);
                                }
                            },
                            _ => {} // Writing to 0x6000 - 0x7fff does nothing
                        }
                    }
                }
            },
            0x8000..=0x9fff => {
                if self.vram_protected {
                    return;
                }

                // Mask to address within range 0x0000-0x1fff and write to that VRAM address
                let masked_address = masked_address & Wrapping(0x1fff);
                self.vram[(self.vram_bank_offset + masked_address).0] = byte;

                // Decode character set from GB format to something more computer-friendly, only if writing within tileset
                if masked_address.0 >= 0x1800 {
                    return;
                }

                // Get the pair of bytes just modified (i.e. one row in the character)
                // Note there are 384 characters in the map, per VRAM bank, each stored with 16 bytes
                let relative_vram_address = masked_address & Wrapping(0x1ffe);
                let byte1 = self.vram[(self.vram_bank_offset + relative_vram_address).0].0 as u32 & 0x000000ff;
                let byte2 = self.vram[(self.vram_bank_offset + relative_vram_address + Wrapping(1)).0].0 as u32 & 0x000000ff;


                // Find the address into decoded data to update now
                // The output format uses 64 bytes per tile rather than 16, hence input address * 4
                let mut output_address = relative_vram_address * Wrapping(4);
                if self.vram_bank_offset.0 != 0 {
                    output_address += 24576;
                }

                // Update the tile set cache in GPU
                self.tile_set[output_address.0] = ((byte2 >> 6) & 0x02) + (byte1 >> 7);
                self.tile_set[(output_address + Wrapping(1)).0] = ((byte2 >> 5) & 0x02) + ((byte1 >> 6) & 0x01);
                self.tile_set[(output_address + Wrapping(2)).0] = ((byte2 >> 4) & 0x02) + ((byte1 >> 5) & 0x01);
                self.tile_set[(output_address + Wrapping(3)).0] = ((byte2 >> 3) & 0x02) + ((byte1 >> 4) & 0x01);
                self.tile_set[(output_address + Wrapping(4)).0] = ((byte2 >> 2) & 0x02) + ((byte1 >> 3) & 0x01);
                self.tile_set[(output_address + Wrapping(5)).0] = ((byte2 >> 1) & 0x02) + ((byte1 >> 2) & 0x01);
                self.tile_set[(output_address + Wrapping(6)).0] = (byte2 & 0x02) + ((byte1 >> 1) & 0x01);
                self.tile_set[(output_address + Wrapping(7)).0] = ((byte2 << 1) & 0x02) + (byte1 & 0x01);
            },
            0xa000..=0xbfff => {
                if let Some(sram) = &mut self.sram {
                    if sram.enable_flag {
                        if sram.has_timer {
                            if sram.timer_mode > 0 {
                                sram.latch_timer_data();
                                sram.write_timer_data(sram.timer_mode as usize, byte);
                                sram.timer_data[sram.timer_mode as usize] = byte;  // TODO - Check this
                            } else if sram.bank_offset.0 < 0x8000 {
                                sram.write_byte(masked_address, byte);
                            }
                        } else {
                            let byte = if self.rom.mbc == Mbc::Mbc2 {
                                byte & Wrapping(0x0f)
                            } else {
                                byte
                            };
                            sram.write_byte(masked_address, byte);
                        }
                    }
                }
            },
            0xc000..=0xcfff => {
                self.wram[(masked_address & Wrapping(0x0fff)).0] = byte;
            },
            0xd000..=0xdfff => {
                self.wram[(self.wram_bank_offset + (masked_address & Wrapping(0x0fff))).0] = byte;
            },
            0xe000..=0xefff => {
                self.wram[(masked_address & Wrapping(0x0fff)).0] = byte;
            },
            0xf000..=0xfdff => {
                self.wram[(self.wram_bank_offset + (masked_address & Wrapping(0x0fff))).0] = byte;
            },
            0xfe00..=0xfe9f => {
                if !self.oam_protected {
                    self.oam[(masked_address & Wrapping(0x00ff)).0 % 160] = byte;
                }
            },
            0xfea0..=0xfeff => {}, // Unusable
            0xff00..=0xff7f => {
                self.write_io(masked_address & Wrapping(0x007f), byte);
            },
            _ => {
                self.io_ports[(masked_address & Wrapping(0x00ff)).0] = byte;
            }
        }
    }

    pub fn read_address_16(&self, address: Wrapping<usize>) -> (Wrapping<u8>, Wrapping<u8>) {
        let msb = self.read_address(address);
        let lsb = self.read_address(address + Wrapping(1));
        (msb, lsb)
    }

    pub fn write_address_16(&mut self, address: Wrapping<usize>, bytes: (Wrapping<u8>, Wrapping<u8>)) {
        self.write_address(address, bytes.0);
        self.write_address(address + Wrapping(1), bytes.1)
    }

    pub fn read_io(&self, address: Wrapping<usize>) -> Wrapping<u8> {
        todo!()
    }

    pub fn write_io(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>) {

    }

    pub fn set_vram_protection(&mut self, protected: bool) {
        self.vram_protected = protected;
    }

    pub fn set_oam_protection(&mut self, protected: bool) {
        self.oam_protected = protected;
    }

    pub fn perform_and(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>) {
        self.write_address(address, self.read_address(address) & byte);
    }

    pub fn perform_or(&mut self, address: Wrapping<usize>, byte: Wrapping<u8>) {
        self.write_address(address, self.read_address(address) | byte);
    }
}
