
use crate::audio::{AudioController, DummyAudioController};
use crate::cpu::Cpu;
use crate::gpu::Gpu;
use crate::input::Input;
use crate::mem::MemBus;
use crate::rom::Rom;
use crate::sgb::Sgb;

use std::cell::RefCell;

const CLOCK_MULTIPLIERS: [i64; 21] = [  1,  1,  1, 1, 1, 2, 1, 4, 2, 4, 1, 5, 3, 7, 2, 5, 3, 5, 8, 12, 20 ];
const CLOCK_DIVISORS: [i64; 21] =    [ 20, 12,  8, 5, 3, 5, 2, 7, 3, 5, 1, 4, 2, 4, 1, 2, 1, 1, 1,  1,  1 ];

pub struct Emulator<A: AudioController> {
    cpu: Cpu,
    mem_bus: RefCell<MemBus>,
    gpu: RefCell<Gpu>,
    sgb: Sgb,
    audio: RefCell<A>,
    input: Input,
    wram: Vec<u8>,
    vram: Vec<u8>,
    io_ports: Vec<u8>,
    divider_count: u32,
    accumulated_clocks: u64,
    clock_multiply: i64,
    clock_divide: i64,
    current_clock_multiplier_combo: usize
}

impl Emulator<DummyAudioController> {

    pub fn load_rom(rom: Rom) -> Self {
        let mut emulator = Self {
            cpu: Cpu::new(rom.get_preferred_hardware_type()),
            mem_bus: RefCell::new(MemBus::new(rom)),
            gpu: RefCell::new(Gpu::new()),
            sgb: Sgb::new(),
            audio: RefCell::new(DummyAudioController::new()),
            input: Input::new(),
            wram: Vec::with_capacity(8 * 4096),
            vram: Vec::with_capacity(2 * 8192),
            io_ports: Vec::with_capacity(256),
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

        let cpu_frequency = self.cpu.get_clock_frequency() as i64;
        let adjusted_frequency =
            (cpu_frequency * self.clock_multiply / self.clock_divide) as f64;
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
        let mut gpu = self.gpu.borrow_mut();
        self.cpu.emulate_clock_cycles::<MemBus, DummyAudioController>(
            self.accumulated_clocks as i64,
            &mut mem_bus,
            &mut audio,
            &mut gpu);
    }

    pub fn reset(&mut self) {

        let cpu_type = self.mem_bus.borrow().rom.get_preferred_hardware_type();

        // Reset component states
        self.cpu.reset(cpu_type);
        self.mem_bus.borrow_mut().reset(cpu_type);
        self.gpu.borrow_mut().reset();
        self.sgb.reset();
        self.audio.borrow_mut().reset(self.cpu.get_clock_frequency());
        self.input.reset();

        // Reset clock multipliers
        self.current_clock_multiplier_combo = 10;
        self.clock_multiply = 1;
        self.clock_divide = 1;
    }
}
