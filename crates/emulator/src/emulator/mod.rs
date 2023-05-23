
use crate::audio::{AudioController, DummyAudioController};
use crate::external_ram::{Sram, ExternalRam};
use crate::input::Input;
use crate::rom::{Rom, Mbc};
use crate::sgb::Sgb;

use std::cell::RefCell;

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
    pub wram: Vec<u8>,
    pub vram: Vec<u8>,
    pub sram: Option<Sram>,
    pub oam: Vec<u8>,
    pub io_ports: Vec<u8>,
    vram_protected: bool,
    oam_protected: bool,
    rom_bank_offset: usize,
    wram_bank_offset: usize,
    vram_bank_offset: usize,
    rom_mbc_mode: u32,

    // CPU
    cpu_type: CpuType,
    pc: usize,
    sp: usize,
    a: u8,
    f: u8,
    b: u8,
    c: u8,
    d: u8,
    e: u8,
    h: u8,
    l: u8,
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
    time_in_current_gpu_mode: u32,
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
            wram: vec![0; 8 * 4096],
            vram: vec![0; 2 * 8192],
            sram,
            oam: vec![0; 160],
            io_ports: vec![0; 256],
            vram_protected: false,
            oam_protected: false,
            rom_bank_offset: 0,
            wram_bank_offset: 0,
            vram_bank_offset: 0,
            rom_mbc_mode: 0,

            cpu_type,
            pc: 0,
            sp: 0,
            a: 0,
            f: 0,
            b: 0,
            c: 0,
            d: 0,
            e: 0,
            h: 0,
            l: 0,
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
            time_in_current_gpu_mode: 0,
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
        self.wram.fill(0);
        self.vram.fill(0);
        self.oam.fill(0);
        self.io_ports.fill(0);

        // Set various state
        self.vram_protected = false;
        self.oam_protected = false;
        self.rom_bank_offset = 0x4000;
        self.wram_bank_offset = 0x1000;
        self.vram_bank_offset = 0x0000;
        self.rom_mbc_mode = 0;

        // Initialise IO ports
        self.io_ports[5] = 0x00;
        self.io_ports[6] = 0x00;
        self.io_ports[7] = 0x00;
        self.io_ports[16] = 0x80;
        self.io_ports[17] = 0xbf;
        self.io_ports[18] = 0xf3;
        self.io_ports[20] = 0xbf;
        self.io_ports[22] = 0x3f;
        self.io_ports[23] = 0x00;
        self.io_ports[25] = 0xbf;
        self.io_ports[26] = 0x7f;
        self.io_ports[27] = 0xff;
        self.io_ports[28] = 0x9f;
        self.io_ports[30] = 0xbf;
        self.io_ports[32] = 0xff;
        self.io_ports[33] = 0x00;
        self.io_ports[34] = 0x00;
        self.io_ports[35] = 0xbf;
        self.io_ports[36] = 0x77;
        self.io_ports[37] = 0xf3;
        self.io_ports[64] = 0x91;
        self.io_ports[66] = 0x00;
        self.io_ports[67] = 0x00;
        self.io_ports[69] = 0x00;
        self.io_ports[71] = 0xfc;
        self.io_ports[72] = 0xff;
        self.io_ports[73] = 0xff;
        self.io_ports[74] = 0x00;
        self.io_ports[75] = 0x00;
        self.io_ports[85] = 0xff;
        self.io_ports[255] = 0x00;

        // Conditional state
        if cpu_type == CpuType::Sgb {
            self.io_ports[38] = 0xf0;
        } else {
            self.io_ports[38] = 0xf1;
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
        self.time_in_current_gpu_mode = 0;
        self.blanked_screen = false;
        self.cgb_bg_pal_index = 0;
        self.cgb_obj_pal_index = 0;

        self.serial_request = false;
        self.serial_is_transferring = false;
        self.serial_clock_is_external = false;
        self.serial_timer = 0;

        // Set various state
        self.pc = 0x0100;
        self.sp = 0xfffe;
        self.f = 0xb0;
        self.b = 0x00;
        self.c = 0x13;
        self.d = 0x00;
        self.e = 0xd8;
        self.h = 0x01;
        self.l = 0x4d;

        // Hardware-dependent state
        match cpu_type {
            CpuType::Dmg => {
                self.a = 0x01;
                self.clock_frequency = GB_FREQ;
            },
            CpuType::Sgb => {
                self.a = 0x11;
                self.clock_frequency = SGB_FREQ;
            },
            CpuType::Cgb => {
                self.a = 0x11;
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

    fn on_invalid_instruction(&mut self) {
        self.is_running = false;
    }

    fn on_display_disabled(&mut self) {
        if self.blanked_screen {
            return;
        }
        self.time_in_current_gpu_mode = 0;
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
            let upper_bits = self.read_address(0xff00) & 0x30;
            if upper_bits == 0x20 {
                self.perform_and(0xff00, 0xf0);
                self.perform_or(0xff00, self.current_key_dir);
            } else if upper_bits == 0x10 {
                self.perform_and(0xff00, 0xf0);
                self.perform_or(0xff00, self.current_key_but);
            }

            // Set interrupt request flag
            self.perform_or(0xff0f, 0x10);
        }

        let mut remaining_clocks = accumulated_clocks;
        while remaining_clocks > 0 {
            let progress = self.emulate_next_step() as i64;
            if progress <= 32 {
                let display_enabled = (self.read_address(0xff40) & 0x80) != 0;
                if display_enabled {
                    self.emulate_gpu(progress);
                } else {
                    if !self.blanked_screen {
                        self.set_oam_protection(false);
                        self.set_vram_protection(false);
                        self.write_address(0xff44, 0);
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
                if self.time_in_current_gpu_mode < 204 {
                    return;
                }
                self.time_in_current_gpu_mode -= 204;
                self.io_ports[0x44] += 1;
                if self.io_ports[0x44] == 144 {
                    self.gpu_mode = GpuMode::VBlank;
                    self.io_ports[0x41] &= 0xfc;
                    self.io_ports[0x41] |= GpuMode::VBlank as u8;
                    // Set interrupt request for VBLANK
                    self.io_ports[0x0f] |= 0x01;
                    self.oam_protected = false;
                    self.vram_protected = false;
                    if (self.io_ports[0x41] & 0x10) != 0x00 {
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
                    if (self.io_ports[0x41] & 0x20) != 0x00 {
                        // Request status int if condition met
                        self.io_ports[0x0f] |= 0x02;
                    }
                }
            },
            GpuMode::VBlank => {
                if self.time_in_current_gpu_mode < 456 {
                    return;
                }
                // 10 of these lines in vblank
                self.time_in_current_gpu_mode -= 456;
                self.io_ports[0x44] += 1;
                if self.io_ports[0x44] >= 154 {
                    self.gpu_mode = GpuMode::ScanOam;
                    self.io_ports[0x41] &= 0xfc;
                    self.io_ports[0x41] |= GpuMode::ScanOam as u8;
                    self.io_ports[0x44] = 0;
                    self.oam_protected = true;
                    self.vram_protected = false;
                    if (self.io_ports[0x41] & 0x20) != 0x00 {
                        // Request status int if condition met
                        self.io_ports[0x0f] |= 0x02;
                    }

                    // LCD starting at top of frame, ready a frame buffer if available
                    // frameManager.beginNewFrame();
                }
            },
            GpuMode::ScanOam => {
                if self.time_in_current_gpu_mode < 80 {
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
                if self.time_in_current_gpu_mode < 172 {
                    return;
                }
                self.time_in_current_gpu_mode -= 172;
                self.gpu_mode = GpuMode::HBlank;
                self.io_ports[0x41] &= 0xfc;
                self.io_ports[0x41] |= GpuMode::HBlank as u8;
                self.oam_protected = false;
                self.vram_protected = false;
                if (self.io_ports[0x41] & 0x08) != 0x00 {
                    // Request status int if condition met
                    self.io_ports[0x0f] |= 0x02;
                }
                // Run DMA if applicable
                if self.io_ports[0x55] < 0xff {
                    // H-blank DMA currently active
                    let mut src_addr = ((self.io_ports[0x51] as usize) << 8) + self.io_ports[0x52] as usize; // DMA source
                    // Don't do transfers within VRAM or from these other addresses either
                    if (src_addr & 0xe000) != 0x8000 && src_addr < 0xe000 {
                        let mut dst_addr = ((self.io_ports[0x53] as usize) << 8) + self.io_ports[0x54] as usize + 0x8000; // DMA destination
                        for count in 0..16 {
                            self.write_address(dst_addr, self.read_address(src_addr));
                            src_addr += 1;
                            dst_addr = (dst_addr + 1) & 0x9fff; // Keep it within VRAM
                        }
                    }

                    // TODO - what is this?
                    //if (ClockFreq == GBC_FREQ) clocks_acc -= 64;
                    //else clocks_acc -= 32;
                    self.io_ports[0x55] -= 1;
                    if self.io_ports[0x55] < 0x80 {
                        // End the DMA
                        self.io_ports[0x55] = 0xff;
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
        let display_enabled = (self.read_address(0xff40) & 0x80) != 0;
        if display_enabled && self.read_address(0xff44) == self.read_address(0xff45) {

            // Set coincidence flag
            self.perform_or(0xff41, 0x04);

            // Request interrupt if his signal goes low to high
            if (self.read_address(0xff41) & 0x40) != 0 && self.last_ly_compare == 0 {
                self.perform_or(0xff0f, 0x02);
            }
            self.last_ly_compare = 1;
        } else {
            self.perform_and(0xff41, 0xfb);
            self.last_ly_compare = 0;
        }

        // Handling of timers
        self.divider_count += clocks_passed_by_instruction as u32;
        if self.divider_count >= 256 {
            self.divider_count -= 256;
            self.write_address(0xff04, self.read_address(0xff04) + 1);
        }
        if self.timer_running {
            self.timer_count += 1;
            if self.timer_count >= self.timer_inc_time {
                self.timer_count -= self.timer_inc_time;
                let new_register = self.read_address(0xff05) + 1;
                self.write_address(0xff05, new_register);
                if new_register == 0 {
                    self.write_address(0xff05, self.read_address(0xff06));
                    self.perform_or(0xff0f, 0x04);
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
                    self.io_ports[0x01] = 0xff;
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

        let triggered_interrupts = self.read_address(0xffff) & self.read_address(0xff0f) & 0x1f;
        if triggered_interrupts == 0 {
            return;
        }

        let mut to_address = self.pc;
        if (triggered_interrupts & 0x01) != 0 {
            // V-blank
            self.perform_and(0xff0f, 0x1e);
            to_address = 0x0040;
        } else if (triggered_interrupts & 0x02) != 0 {
            // LCD stat
            self.perform_and(0xff0f, 0x1d);
            to_address = 0x0048;
        } else if (triggered_interrupts & 0x04) != 0 {
            // Timer
            self.perform_and(0xff0f, 0x1b);
            to_address = 0x0050;
        } else if (triggered_interrupts & 0x08) != 0 {
            // Serial
            self.perform_and(0xff0f, 0x17);
            to_address = 0x0058;
        } else if (triggered_interrupts & 0x10) != 0 {
            // Joypad
            self.perform_and(0xff0f, 0x0f);
            to_address = 0x0060;
        }

        // Unless halted with IME unset, push PC onto stack and go to interrupt handler address
        if !cpu_halted || self.ime {
            self.sp -= 2;
            self.write_address_16(
                self.sp as usize, ((self.pc & 0xff) as u8, (self.pc >> 8) as u8));
            self.pc = to_address;
        }
        self.mode = Mode::Running;
        self.ime = false;
    }

    #[inline]
    fn switch_running_speed(&mut self) -> bool {
        let speed_change_requested = self.cpu_type == CpuType::Cgb && self.read_address(0xff4d) == 0x01;
        if speed_change_requested {
            // Speed change was requested in CGB mode
            self.perform_and(0xff4d, 0x80);
            let new_byte = self.read_address(0xff4d);
            if new_byte == 0x00 {
                self.write_address(0xff4d, 0x80);
                self.clock_frequency = CGB_FREQ;
                self.gpu_clock_factor = 2;
                return true;
            } else {
                self.write_address(0xff4d, 0x00);
                self.clock_frequency = GB_FREQ;
                self.gpu_clock_factor = 1;
                return true;
            }
        }
        false
    }

    #[inline]
    fn perform_op(&mut self) -> u64 {
        todo!()
    }

    pub fn read_address(&self, address: usize) -> u8 {
        let masked_address = address & 0xffff;
        match masked_address {
            0x0000..=0x3fff => self.rom.data[masked_address],
            0x4000..=0x7fff => self.rom.data[self.rom_bank_offset + masked_address],
            0x8000..=0x9fff => self.vram[self.vram_bank_offset + masked_address - 0x8000],
            0xa000..=0xbfff => {
                if let Some(sram) = &self.sram {
                    if sram.has_timer && sram.timer_mode > 0 {
                        // TODO - Check this bit
                        sram.timer_data[sram.timer_mode as usize - 0x08]
                    } else {
                        sram.read_byte(masked_address - 0xa000)
                    }
                } else {
                    0xff
                }
            },
            0xc000..=0xcfff => self.wram[masked_address - 0xc000],
            0xd000..=0xdfff => self.wram[self.wram_bank_offset + masked_address - 0xd000],
            0xe000..=0xefff => self.wram[masked_address - 0xe000],
            0xf000..=0xfdff => self.wram[self.wram_bank_offset + masked_address - 0xf000],
            0xfe00..=0xfe9f => {
                if self.oam_protected {
                    0xff
                } else {
                    self.oam[(masked_address & 0x00ff) % 160]
                }
            },
            0xfea0..=0xfeff => 0xff,
            0xff00..=0xff7f => self.read_address(masked_address & 0x007f),
            _ => self.io_ports[masked_address & 0x00ff]
        }
    }

    pub fn write_address(&mut self, address: usize, byte: u8) {
        let masked_address = address & 0xffff;
        match masked_address {
            0x0000..=0x7fff => {
                match self.rom.mbc {
                    Mbc::None => {},
                    Mbc::Mbc1 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                // Only 4 bits are used. Writing 0xa enables SRAM, anything else disables it
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2000..=0x3fff => {
                                // Set low 5 bits of bank number
                                self.rom_bank_offset &= 0xfff80000;
                                let mut byte = byte & 0x1f;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset |= byte as usize * 0x4000;
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                let byte = byte & 0x03;
                                if let Some(sram) = &mut self.sram {
                                    if self.rom_mbc_mode != 0 {
                                        sram.bank_offset = byte as usize * 0x2000; // Select RAM bank
                                    } else {
                                        self.rom_bank_offset &= 0xffe7c000;
                                        self.rom_bank_offset |= byte as usize * 0x80000;
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
                                        self.rom_mbc_mode = byte as u32 & 0x01;
                                    }
                                }
                            }
                        }
                    },
                    Mbc::Mbc2 => {
                        match masked_address {
                            0x0000..=0x0fff => {
                                if let Some(sram) = &mut self.sram {
                                    // Only 4 bits are used. Writing 0xa enables SRAM.
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2100..=0x21fe => {
                                let mut byte = (byte & 0x0f) & self.rom.bank_select_mask as u8;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset = byte as usize * 0x4000;
                            },
                            _ => {}
                        }
                    },
                    Mbc::Mbc3 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a; // Also enables timer registers
                                }
                            },
                            0x2000..=0x3fff => {
                                let mut byte = byte & self.rom.bank_select_mask as u8;
                                if byte == 0x00 {
                                    byte = 0x01;
                                }
                                self.rom_bank_offset = byte as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & 0x0f;
                                    if byte < 0x04 {
                                        sram.bank_offset = byte as usize * 0x2000;
                                        sram.timer_mode = 0;
                                    } else if (byte >= 0x08) && (byte < 0x0d) {
                                        sram.timer_mode = byte as u32;
                                    } else {
                                        sram.timer_mode = 0;
                                    }
                                }
                            },
                            _ => {
                                if let Some(sram) = &mut self.sram {
                                    let byte = byte & 0x01;
                                    if (sram.timer_latch == 0x00) && (byte == 0x01) {
                                        sram.latch_timer_data();
                                    }
                                    sram.timer_latch = byte as u32;
                                }
                            }
                        }
                    },
                    Mbc::Mbc5 => {
                        match masked_address {
                            0x0000..=0x1fff => {
                                // RAMG - 4 bits, enable external RAM by writing 0xa
                                if let Some(sram) = &mut self.sram {
                                    sram.enable_flag = (byte & 0x0f) == 0x0a;
                                }
                            },
                            0x2000..=0x2fff => {
                                // ROMB0 - lower 8 bits of 9-bit ROM bank (note MBC5 can select bank 0 here)
                                let masked_byte = byte as usize & self.rom.bank_select_mask as usize;
                                self.rom_bank_offset = (self.rom_bank_offset & 0x00400000) | (masked_byte * 0x4000);
                            },
                            0x3000..=0x3fff => {
                                // ROMB1 - 1 bit, upper bit of 9-bit RAM bank (note MBC5 can select bank 0 here)
                                self.rom_bank_offset &= 0x003fc000;
                                if (byte & 0x01) != 0x00 {
                                    self.rom_bank_offset |= 0x00004000;
                                }
                                self.rom_bank_offset &= self.rom.bank_select_mask as usize * 0x4000;
                            },
                            0x4000..=0x5fff => {
                                // RAMB - 4-bit RAM bank
                                if let Some(sram) = &mut self.sram {
                                    sram.bank_offset = (byte & 0x0f) as usize * 0x2000;
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
                let masked_address = masked_address & 0x1fff;
                self.vram[self.vram_bank_offset + masked_address] = byte;

                // Decode character set from GB format to something more computer-friendly, only if writing within tileset
                if masked_address >= 0x1800 {
                    return;
                }

                // Get the pair of bytes just modified (i.e. one row in the character)
                // Note there are 384 characters in the map, per VRAM bank, each stored with 16 bytes
                let relative_vram_address = masked_address & 0x1ffe;
                let byte1 = self.vram[self.vram_bank_offset + relative_vram_address] as u32 & 0x000000ff;
                let byte2 = self.vram[self.vram_bank_offset + relative_vram_address + 1] as u32 & 0x000000ff;


                // Find the address into decoded data to update now
                // The output format uses 64 bytes per tile rather than 16, hence input address * 4
                let mut output_address = relative_vram_address * 4;
                if self.vram_bank_offset != 0 {
                    output_address += 24576;
                }

                // Update the tile set cache in GPU
                self.tile_set[output_address] = ((byte2 >> 6) & 0x02) + (byte1 >> 7);
                self.tile_set[output_address + 1] = ((byte2 >> 5) & 0x02) + ((byte1 >> 6) & 0x01);
                self.tile_set[output_address + 2] = ((byte2 >> 4) & 0x02) + ((byte1 >> 5) & 0x01);
                self.tile_set[output_address + 3] = ((byte2 >> 3) & 0x02) + ((byte1 >> 4) & 0x01);
                self.tile_set[output_address + 4] = ((byte2 >> 2) & 0x02) + ((byte1 >> 3) & 0x01);
                self.tile_set[output_address + 5] = ((byte2 >> 1) & 0x02) + ((byte1 >> 2) & 0x01);
                self.tile_set[output_address + 6] = (byte2 & 0x02) + ((byte1 >> 1) & 0x01);
                self.tile_set[output_address + 7] = ((byte2 << 1) & 0x02) + (byte1 & 0x01);
            },
            0xa000..=0xbfff => {
                if let Some(sram) = &mut self.sram {
                    if sram.enable_flag {
                        if sram.has_timer {
                            if sram.timer_mode > 0 {
                                sram.latch_timer_data();
                                sram.write_timer_data(sram.timer_mode as usize, byte);
                                sram.timer_data[sram.timer_mode as usize] = byte;  // TODO - Check this
                            } else if sram.bank_offset < 0x8000 {
                                sram.write_byte(masked_address, byte);
                            }
                        } else {
                            let byte = if self.rom.mbc == Mbc::Mbc2 {
                                byte & 0x0f
                            } else {
                                byte
                            };
                            sram.write_byte(masked_address, byte);
                        }
                    }
                }
            },
            0xc000..=0xcfff => {
                self.wram[masked_address & 0x0fff] = byte;
            },
            0xd000..=0xdfff => {
                self.wram[self.wram_bank_offset + (masked_address & 0x0fff)] = byte;
            },
            0xe000..=0xefff => {
                self.wram[masked_address & 0x0fff] = byte;
            },
            0xf000..=0xfdff => {
                self.wram[self.wram_bank_offset + (masked_address & 0x0fff)] = byte;
            },
            0xfe00..=0xfe9f => {
                if !self.oam_protected {
                    self.oam[(masked_address & 0x00ff) % 160] = byte;
                }
            },
            0xfea0..=0xfeff => {}, // Unusable
            0xff00..=0xff7f => {
                self.write_io(masked_address & 0x007f, byte);
            },
            _ => {
                self.io_ports[masked_address & 0x00ff] = byte;
            }
        }
    }

    pub fn read_address_16(&self, address: usize) -> (u8, u8) {
        todo!()
    }

    pub fn write_address_16(&mut self, address: usize, bytes: (u8, u8)) {
        todo!()
    }

    pub fn read_io(&self, address: usize) -> u8 {
        todo!()
    }

    pub fn write_io(&mut self, address: usize, byte: u8) {

    }

    pub fn set_vram_protection(&mut self, protected: bool) {
        self.vram_protected = protected;
    }

    pub fn set_oam_protection(&mut self, protected: bool) {
        self.oam_protected = protected;
    }

    pub fn perform_and(&mut self, address: usize, byte: u8) {
        self.write_address(address, self.read_address(address) & byte);
    }

    pub fn perform_or(&mut self, address: usize, byte: u8) {
        self.write_address(address, self.read_address(address) | byte);
    }
}
