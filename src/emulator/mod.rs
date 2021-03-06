
mod cpu;
mod external_ram;
mod gpu;
pub mod rom;
mod mem;
mod sgb;
mod audio;
mod input;

use cpu::Cpu;
use mem::MemBus;
use gpu::Gpu;
use rom::Rom;
use sgb::Sgb;
use audio::{AudioController, DummyAudioController};
use input::Input;

use std::cell::RefCell;

const GB_FREQ: u64 = 4194304;
const SGB_FREQ: u64 = 4295454;
const CGB_FREQ: u64 = 8400000;

pub struct Emulator<A: AudioController> {
    cpu: Cpu,
    mem_bus: RefCell<MemBus>,
    gpu: Gpu,
    sgb: Sgb,
    audio: RefCell<A>,
    input: Input,
    wram: Vec<u8>,
    vram: Vec<u8>,
    io_ports: Vec<u8>,
    cpu_clock_frequency: u64,
    divider_count: u32,
    accumulated_clocks: u64,
    clock_multiply: i64,
    clock_divide: i64,
    current_clock_multiplier_combo: i32
}

impl Emulator<DummyAudioController> {

    pub fn load_rom(rom: Rom) -> Self {
        let mut emulator = Self {
            cpu: Cpu::new(rom.cgb_flag),
            mem_bus: RefCell::new(MemBus::new(rom)),
            gpu: Gpu::new(),
            sgb: Sgb::new(),
            audio: RefCell::new(DummyAudioController::new()),
            input: Input::new(),
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            io_ports: Vec::with_capacity(256),
            cpu_clock_frequency: GB_FREQ,
            divider_count: 1,
            accumulated_clocks: 0,
            clock_multiply: 1,
            clock_divide: 1,
            current_clock_multiplier_combo: 10
        };

        emulator
    }

    pub fn start(&mut self) {

    }

    pub fn stop(&mut self) {

    }

    fn do_work(&mut self, time_diff_millis: u64) {

        let frequency = self.cpu_clock_frequency as i64;
        let adjusted_frequency =
            (frequency * self.clock_multiply / self.clock_divide) as f64;
        self.accumulated_clocks =
            self.accumulated_clocks + (time_diff_millis as f64 * 0.001 * adjusted_frequency) as u64;
        let approx_multiplier = (self.clock_multiply / self.clock_divide + 1) as u64;
        let clocks_cap = 1000000 * approx_multiplier;
        if self.accumulated_clocks > clocks_cap {
            self.accumulated_clocks = clocks_cap;
        }

        self.cpu.set_input_values(self.input.key_dir, self.input.key_but);

        let mut mem_bus = self.mem_bus.borrow_mut();
        let mut audio = self.audio.borrow_mut();
        self.cpu.execute_accumulated_clocks::<MemBus, DummyAudioController>(
            self.accumulated_clocks,
            &mut mem_bus,
            &mut audio);
    }

    pub fn reset(&mut self) {

        let (sgb_flag, cgb_flag) = {
            let rom = &self.mem_bus.borrow().rom;
            (rom.sgb_flag, rom.cgb_flag)
        };

        // Reset component states
        self.cpu.reset(cgb_flag);
        self.mem_bus.borrow_mut().reset(sgb_flag);
        self.gpu.reset();
        self.sgb.reset();
        self.audio.borrow_mut().reset(GB_FREQ);
        self.input.reset();

        // Reset clock multipliers
        self.current_clock_multiplier_combo = 10;
        self.clock_multiply = 1;
        self.clock_divide = 1;

        // Conditional state
        if cgb_flag {
            self.cpu_clock_frequency = GB_FREQ;
        } else if sgb_flag {
            self.cpu_clock_frequency = SGB_FREQ;
        } else {
            self.cpu_clock_frequency = GB_FREQ;
        }
    }
}
